/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  (C) Copyright 2024 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <gclib.h>
#include <gclibo.h>

#include <cisstCommon/cmnPath.h>
#include <cisstCommon/cmnAssert.h>

#include <sawGalilController/mtsGalilController.h>

enum GALIL_STATES { ST_IDLE, ST_HOMING };

//****** Axis Data structures in DR packet ******

#pragma pack(push, 1)     // Eliminate structure padding

// AxisDataMin supported by all Galil DMC controllers
//   - GDataRecord4000 (DMC 4000, 4200, 4103, and 500x0)
//   - GDataRecord52000 (DMC 52000)
//   - GDataRecord1806 (DMC 1806)
//   - GDataRecord2103 (DMC 2103)
//   - GDataRecord1802 (DMC 1802)
//   - GDataRecord30000 (DMC 30010)
//
// Galil User Manual states: "The velocity information that is returned in the data
// record is 64 times larger than the value returned when using the command TV (Tell Velocity)"

struct AxisDataMin {
    uint16_t status;
    uint8_t  switches;
    uint8_t  stop_code;
    int32_t  ref_pos;
    int32_t  pos;
    int32_t  pos_error;
    int32_t  aux_pos;
    int32_t  vel;
    int32_t  torque;
    uint16_t analog_in;   // reserved for 1802
};

// AxisDataMax supported by:
//   - GDataRecord4000 (DMC 4000, 4200, 4103, and 500x0)
//   - GDataRecord52000 (DMC 52000)
//   - GDataRecord30000 (DMC 30010)
struct AxisDataMax : public AxisDataMin {
    uint8_t  hall;        // reserved for 1806
    uint8_t  reserved;
    int32_t  var;         // User-defined (ZA)
};

#pragma pack(pop)

// Bit masks for AxisData fields
// For a full list, see Galil User Manual

const uint16_t StatusMotorMoving     = 0x8000;
const uint16_t StatusFindEdgeActive  = 0x1000;
const uint16_t StatusHomeActive      = 0x0800;
const uint16_t StatusHome1Done       = 0x0400;
const uint16_t StatusHome2DoneFI     = 0x0200;
const uint16_t StatusHome3Active     = 0x0002;
const uint16_t StatusMotorOff        = 0x0001;

const uint8_t  SwitchFwdLimit        = 0x08;
const uint8_t  SwitchRevLimit        = 0x04;
const uint8_t  SwitchHome            = 0x02;

// Bit masks for Amplifier Status
const uint32_t AmpEloUpper          = 0x02000000;  // ELO active (axes E-H)
const uint32_t AmpEloLower          = 0x01000000;  // ELO active (axes A-D)
const uint32_t AmpPeakCurrentA      = 0x00010000;  // Peak current for axis A (left shift for B-H)
const uint32_t AmpHallErrorA        = 0x00000100;  // Hall error for axis A (left shift for B-h)
const uint32_t AmpUnderVoltageUpper = 0x00000080;  // Under-voltage (axes E-H)
const uint32_t AmpOverTempUpper     = 0x00000040;  // Over-temperature (axes E-H)
const uint32_t AmpOverVoltageUpper  = 0x00000020;  // Over-voltage (axes E-H)
const uint32_t AmpOverCurrentUpper  = 0x00000010;  // Over-current (axes E-H)
const uint32_t AmpUnderVoltageLower = 0x00000008;  // Under-voltage (axes A-D)
const uint32_t AmpOverTempLower     = 0x00000004;  // Over-temperature (axes A-D)
const uint32_t AmpOverVoltageLower  = 0x00000002;  // Over-voltage (axes A-D)
const uint32_t AmpOverCurrentLower  = 0x00000001;  // Over-current (axes A-D)

// Stop codes (see SC command for full list)
const uint8_t SC_Running  =  0;   // Motors are running
const uint8_t SC_Stopped  =  1;   // Motors decelerating or stopped at position
const uint8_t SC_FwdLim   =  2;   // Stopped at forward limit switch (or FL)
const uint8_t SC_RevLim   =  3;   // Stopped at reverse limit switch (or BL)
const uint8_t SC_StopCmd  =  4;   // Stopped by Stop command (ST)
const uint8_t SC_OnError  =  8;   // Stopped by Off on Error (OE)
const uint8_t SC_FindEdge =  9;   // Stopped after finding edge (FE)
const uint8_t SC_Homing   = 10;   // Stopped after homing (HM) or find index (FI)

// Following is information specific to the different Galil DMC controller models.
// There currently are 6 different DMC model types. We do not support any RIO controllers.
// Note also the Galil QZ command, which returns information about the DR structure.
const size_t NUM_MODELS = 6;
const size_t ADmin = sizeof(AxisDataMin);
const size_t ADmax = sizeof(AxisDataMax);
// The Galil model types (corresponding to the different GDataRecord structs)
const unsigned int ModelTypes[NUM_MODELS]     = {  4000, 52000,  1806,  2103,  1802, 30000 };
// Byte offset to the start of the axis data
const unsigned int AxisDataOffset[NUM_MODELS] = {    82,    82,    78 ,   44,    40,    38 };
// Size of the axis data
const size_t AxisDataSize[NUM_MODELS]         = { ADmax, ADmax, ADmin, ADmin, ADmin, ADmax };
// Whether the first 4 bytes contain header information
// For DMC-4143, the header bytes are: 135 (0x87), 15 (0x0f), 226 , 0
//   0x87 MSB always set; 7 indicates that I (Input), T (T Plane) and S (S Plane) blocks present
//   0x0f indicates that blocks (axes) A-D are present, but not E-H
//   last two bytes (swapped) are the size of the data record (226 bytes for DMC-4143)
const bool HasHeader[NUM_MODELS]              = {  true,  true, false,  true, false,  true };
// Byte offset to the sample number
const unsigned int SampleOffset[NUM_MODELS]   = {     4,     4,     0,     4,     0,     4 };
// Byte offset to the error code
const unsigned int ErrorCodeOffset[NUM_MODELS] = {   50,    50,    46,    26,    22,    10 };
// Byte offset to amplifier status (-1 means not available)
const int AmpStatusOffset[NUM_MODELS]          = {   52,    52,    -1,    -1,    -1,    18 };

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsGalilController, mtsTaskContinuous, mtsStdString)

mtsGalilController::mtsGalilController(const std::string &name) :
    mtsTaskContinuous(name, 1024, true), mGalil(0), mHeader(0), mAmpStatus(0),
    mMotorPowerOn(false), mMotionActive(false), mState(ST_IDLE)
{
    Init();
}

mtsGalilController::mtsGalilController(const std::string &name, unsigned int sizeStateTable, bool newThread) :
    mtsTaskContinuous(name, sizeStateTable, newThread), mGalil(0), mHeader(0), mAmpStatus(0),
    mMotorPowerOn(false), mMotionActive(false), mState(ST_IDLE)
{
    Init();
}

mtsGalilController::mtsGalilController(const mtsTaskContinuousConstructorArg & arg) :
    mtsTaskContinuous(arg), mGalil(0), mHeader(0),mAmpStatus(0),  mMotorPowerOn(false), mMotionActive(false),
    mState(ST_IDLE)
{
    Init();
}

mtsGalilController::~mtsGalilController()
{
    Close();
    delete [] mBuffer;
}

void mtsGalilController::Init(void)
{
    // Call SetupInterfaces after Configure, for reasons documented below
    // (see comment at end of Configure method).
    mBuffer = new char[G_SMALL_BUFFER];
}

void mtsGalilController::SetupInterfaces(void)
{
    StateTable.AddData(mHeader, "dr_header");
    StateTable.AddData(mSampleNum, "sample_num");
    StateTable.AddData(mErrorCode, "error_code");
    StateTable.AddData(m_measured_js, "measured_js");
    StateTable.AddData(m_setpoint_js, "setpoint_js");
    StateTable.AddData(m_op_state, "op_state");
    StateTable.AddData(mAxisStatus, "axis_status");
    StateTable.AddData(mStopCode, "stop_code");
    StateTable.AddData(mSwitches, "switches");
    StateTable.AddData(mAnalogIn, "analog_in");
    StateTable.AddData(mActuatorState, "actuator_state");
    StateTable.AddData(mSpeed, "speed");
    StateTable.AddData(mAccel, "accel");
    StateTable.AddData(mDecel, "decel");

    mInterface = AddInterfaceProvided("control");
    if (mInterface) {
        // for Status, Warning and Error with mtsMessage
        mInterface->AddMessageEvents();

        // Standard CRTK interfaces
        mInterface->AddCommandReadState(this->StateTable, m_measured_js, "measured_js");
        mInterface->AddCommandReadState(this->StateTable, m_setpoint_js, "setpoint_js");
        mInterface->AddCommandReadState(this->StateTable, m_op_state, "operating_state");  // TODO
        mInterface->AddCommandWrite(&mtsGalilController::servo_jp, this, "servo_jp");
        mInterface->AddCommandWrite(&mtsGalilController::servo_jr, this, "servo_jr");
        mInterface->AddCommandWrite(&mtsGalilController::servo_jv, this, "servo_jv");
        mInterface->AddCommandVoid(&mtsGalilController::hold, this, "hold");
        mInterface->AddCommandRead(&mtsGalilController::GetConfig_js, this, "configuration_js");

        mInterface->AddCommandVoid(&mtsGalilController::EnableMotorPower, this, "EnableMotorPower");
        mInterface->AddCommandVoid(&mtsGalilController::DisableMotorPower, this, "DisableMotorPower");

        // TEMP: following is to be able to use prmStateRobotQtWidgetComponent
        mInterface->AddCommandRead(&mtsGalilController::measured_cp, this, "measured_cp");

        // Stats
        mInterface->AddCommandReadState(StateTable, StateTable.PeriodStats, "period_statistics");

        // Extra stuff
        mInterface->AddCommandRead(&mtsGalilController::GetNumAxes, this, "GetNumAxes");
        mInterface->AddCommandRead(&mtsGalilController::GetHeader, this, "GetHeader");
        mInterface->AddCommandReadState(this->StateTable, mSampleNum, "GetSampleNum");
        mInterface->AddCommandReadState(this->StateTable, mErrorCode, "GetErrorCode");
        mInterface->AddCommandRead(&mtsGalilController::GetConnected, this, "GetConnected");
        mInterface->AddCommandWrite(&mtsGalilController::SendCommand, this, "SendCommand");
        mInterface->AddCommandWriteReturn(&mtsGalilController::SendCommandRet, this, "SendCommandRet");
        mInterface->AddCommandReadState(this->StateTable, mAnalogIn, "GetAnalogInput");
        mInterface->AddCommandVoid(&mtsGalilController::AbortProgram, this, "AbortProgram");
        mInterface->AddCommandVoid(&mtsGalilController::AbortMotion, this, "AbortMotion");
        mInterface->AddCommandWrite(&mtsGalilController::SetSpeed, this, "SetSpeed");
        mInterface->AddCommandWrite(&mtsGalilController::SetAccel, this, "SetAccel");
        mInterface->AddCommandWrite(&mtsGalilController::SetDecel, this, "SetDecel");
        mInterface->AddCommandWrite(&mtsGalilController::Home, this, "Home");
        mInterface->AddCommandWrite(&mtsGalilController::UnHome, this, "UnHome");
        mInterface->AddCommandWrite(&mtsGalilController::FindEdge, this, "FindEdge");
        mInterface->AddCommandWrite(&mtsGalilController::FindIndex, this, "FindIndex");
        mInterface->AddCommandWrite(&mtsGalilController::SetHomePosition, this, "SetHomePosition");
        mInterface->AddCommandReadState(this->StateTable, mActuatorState, "GetActuatorState");
        mInterface->AddCommandReadState(this->StateTable, mSpeed, "GetSpeed");
        mInterface->AddCommandReadState(this->StateTable, mAccel, "GetAccel");
        mInterface->AddCommandReadState(this->StateTable, mDecel, "GetDecel");
        // Low-level axis data for testing
        mInterface->AddCommandReadState(this->StateTable, mAxisStatus, "GetAxisStatus");
        mInterface->AddCommandReadState(this->StateTable, mStopCode, "GetStopCode");
        mInterface->AddCommandReadState(this->StateTable, mSwitches, "GetSwitches");
    }
}

void mtsGalilController::Close()
{

    if (mGalil) {
        GClose(mGalil);
        mGalil = 0;
    }
}

unsigned int mtsGalilController::GetModelIndex(unsigned int modelType)
{
    unsigned int i;
    for (i = 0; i < NUM_MODELS; i++) {
        if (modelType == ModelTypes[i]) {
            break;
        }
    }
    return i;
}

void mtsGalilController::Configure(const std::string& fileName)
{
    std::string dmcStartupFile;

    std::ifstream jsonStream;
    jsonStream.open(fileName.c_str());
    Json::Value jsonConfig;
    Json::Reader jsonReader;
    if (!jsonReader.parse(jsonStream, jsonConfig)) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: failed to parse " << fileName << " for Galil config" << std::endl
                                 << jsonReader.getFormattedErrorMessages();
        exit(EXIT_FAILURE);
    }
    try {
        m_configuration.DeSerializeTextJSON(jsonConfig);
    } catch (std::exception & std_exception) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: " << fileName << ": " << std_exception.what() << std::endl;
        exit(EXIT_FAILURE);
    }   

    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: parsed file " << fileName << std::endl
                               << "Loaded configuration:" << std::endl
                               << m_configuration << std::endl;
    
    // Size of array determines number of axes
    mNumAxes = m_configuration.axes.size();
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: found " << mNumAxes << " axes" << std::endl;

    mModel = GetModelIndex(m_configuration.model);
    if (mModel < NUM_MODELS) {
        CMN_LOG_CLASS_INIT_VERBOSE << "Configure: setting Galil model to " << m_configuration.model
                                   << " (index = " << mModel << ")" << std::endl;
    }

    // Now, set the data sizes
    m_config_j.Name().SetSize(mNumAxes);
    m_config_j.Type().SetSize(mNumAxes);
    m_config_j.PositionMin().SetSize(mNumAxes);
    m_config_j.PositionMax().SetSize(mNumAxes);
    // We have position and velocity for measured_js
    m_measured_js.Name().SetSize(mNumAxes);
    m_measured_js.Position().SetSize(mNumAxes);
    m_measured_js.Velocity().SetSize(mNumAxes);
    m_measured_js.Position().SetAll(0.0);
    m_measured_js.Velocity().SetAll(0.0);
    // We have position and effort for setpoint_js
    m_setpoint_js.Name().SetSize(mNumAxes);
    m_setpoint_js.Position().SetSize(mNumAxes);
    m_setpoint_js.Effort().SetSize(mNumAxes);
    m_setpoint_js.Position().SetAll(0.);
    m_setpoint_js.Effort().SetAll(0.0);

    mActuatorState.SetSize(mNumAxes);
    mActuatorState.Position().SetAll(0.0);
    mActuatorState.Velocity().SetAll(0.0);

    mAxisToGalilIndexMap.SetSize(mNumAxes);
    mGalilIndexToAxisMap.SetSize(GALIL_MAX_AXES);
    mGalilIndexToAxisMap.SetAll(mNumAxes);   // Initialize to invalid value
    mEncoderCountsPerUnit.SetSize(mNumAxes);
    mEncoderOffset.SetSize(mNumAxes);
    mHomePos.SetSize(mNumAxes);
    mHomeLimitDisable.SetSize(mNumAxes);
    mLimitDisable.SetSize(mNumAxes);
    mAxisStatus.SetSize(mNumAxes);
    mStopCode.SetSize(mNumAxes);
    mSwitches.SetSize(mNumAxes);
    mAnalogIn.SetSize(mNumAxes);

    mSpeed.SetSize(mNumAxes);
    mSpeedDefault.SetSize(mNumAxes);
    mAccel.SetSize(mNumAxes);
    mAccelDefault.SetSize(mNumAxes);
    mDecel.SetSize(mNumAxes);
    mDecelDefault.SetSize(mNumAxes);

    mGalilIndexMax = 0;
    unsigned int i;
    for (i = 0; i < GALIL_MAX_AXES; i++)
        mGalilIndexValid[i] = false;

    for (unsigned int axis = 0; axis < mNumAxes; axis++) {
        sawGalilControllerConfig::axis &axisData = m_configuration.axes[axis];
        mGalilIndexValid[axisData.index] = true;
        mAxisToGalilIndexMap[axis] = axisData.index;
        mGalilIndexToAxisMap[axisData.index] = axis;
        char galilChannel = 'A'+axisData.index;
        if (axisData.index > mGalilIndexMax)
            mGalilIndexMax = axisData.index;   // Save largest Galil index for future efficiency
        m_measured_js.Name()[axis].assign(1, galilChannel);
        m_setpoint_js.Name()[axis].assign(1, galilChannel);
        m_config_j.Name()[axis].assign(1, galilChannel);
        m_config_j.Type()[axis] = static_cast<prmJointType>(axisData.type);
        m_config_j.PositionMin()[axis] = axisData.position_limits.lower;
        m_config_j.PositionMax()[axis] = axisData.position_limits.upper;
        mEncoderCountsPerUnit[axis] = axisData.position_bits_to_SI.scale;
        mEncoderOffset[axis] = static_cast<long>(axisData.position_bits_to_SI.offset);
        mHomePos[axis] = axisData.home_pos;
        mHomeLimitDisable[axis] = 0;
        if (axisData.home_pos <= axisData.position_limits.lower)
            mHomeLimitDisable[axis] |= 2;   // Disable lower limit switch
        else if (axisData.home_pos >= axisData.position_limits.upper)
            mHomeLimitDisable[axis] |= 1;   // Disable upper limit switch

    }
    mGalilIndexMax++;   // Increment so that we can test for less than

    unsigned int k = 0;
    unsigned int q = 0;
    for (i = 0; i < mGalilIndexMax; i++) {
        // If valid axis, add to mGalilAxes
        if (mGalilIndexValid[i]) {
            mGalilAxes[k++] = 'A'+i;
            mGalilQuery[q++] = '?';
        }
        mGalilQuery[q++] = ',';
    }
    mGalilAxes[k] = 0;           // NULL termination
    mGalilQuery[q-1] = 0;        // NULL termination (and remove last comma)

    // Default values should be read from JSON file
    mSpeedDefault.SetAll(0.025);   // 25 mm/s
    mAccelDefault.SetAll(0.256);   // 256 mm/s^2
    mDecelDefault.SetAll(0.256);   // 256 mm/s^2

    // Call SetupInterfaces after Configure because we need to know the correct sizes of
    // the dynamic vectors, which are based on the number of configured axes.
    // These sizes should be set before calling StateTable.AddData and AddCommandReadState;
    // in the latter case, this ensures that the argument prototype has the correct size.
    SetupInterfaces();
}

void mtsGalilController::Startup()
{
    std::string GalilString = m_configuration.IP_address;
    if (m_configuration.direct_mode) {
        GalilString.append(" -d");
    }
    GalilString.append(" -s DR");  // Subscribe to DR records
    GReturn ret = GOpen(GalilString.c_str(), &mGalil);
    if (ret != G_NO_ERROR) {
        mInterface->SendError(this->GetName() + ": error opening " + m_configuration.IP_address);
        CMN_LOG_CLASS_INIT_ERROR << "Galil GOpen: error opening " << m_configuration.IP_address
                                 << ": " << ret << std::endl;
        return;
    }

    // Upload a DMC program file if available
    const std::string & DMC_file = m_configuration.DMC_file;
    if (!DMC_file.empty()) {
        if (cmnPath::Exists(DMC_file)) {
            CMN_LOG_CLASS_INIT_VERBOSE << "Startup: downloading " << DMC_file << " to Galil controller" << std::endl;
            if (GProgramDownloadFile(mGalil, DMC_file.c_str(), 0) == G_NO_ERROR) {
                SendCommand("XQ");  // Execute downloaded program
            }
            else {
                CMN_LOG_CLASS_INIT_ERROR << "Startup: error downloading DMC program file "
                                         << DMC_file << std::endl;
            }
        }
        else {
            CMN_LOG_CLASS_INIT_ERROR << "Startup: DMC program file \""
                                     << DMC_file << "\" not found" << std::endl;
        }
    }

    // Set default speed, accel, decel
    SetSpeed(mSpeedDefault);
    SetAccel(mAccelDefault);
    SetDecel(mDecelDefault);

    // Store the current setting of limit disable (LD) in mLimitDisable
    mLimitDisable.SetAll(0);
    if (!QueryCmdValues("LD ", mGalilQuery, mLimitDisable))
        CMN_LOG_CLASS_INIT_ERROR << "Startup: Could not query limit disable (LD)" << std::endl;
    // Update mHomeLimitDisable based on mLimitDisable
    for (size_t i = 0; i < mNumAxes; i++)
        mHomeLimitDisable[i] |= mLimitDisable[i];

    // Get controller type (^R^V)
    if (GCmdT(mGalil, "\x12\x16", mBuffer, G_SMALL_BUFFER, 0) == G_NO_ERROR) {
        mInterface->SendStatus("Galil Controller Revision: " + std::string(mBuffer));
        unsigned int autoModel = 0;   // detected model type
        const char *ptr = strstr(mBuffer, "DMC");
        if (ptr) {
            ptr += 3;   // Skip DMC
            if ((ptr[0] == '4') || ((ptr[0] == '5') && (ptr[1] == '0')))
                autoModel = 4000;    // 4000, 4200, 4103, and 500x0
            else if ((ptr[0] == '5') && (ptr[1] == '2'))
                autoModel = 52000;   // 52000
            else if (ptr[0] == '3')
                autoModel = 30000;   // 30010
            else if (ptr[0] == '2')
                autoModel = 2103;    // 30010
            else if (strncmp(ptr, "1806", 4) == 0)
                autoModel = 1806;    // 1806
            else if (strncmp(ptr, "1802", 4) == 0)
                autoModel = 1802;    // 1802
        }
        if (mModel >= NUM_MODELS) {
            if (autoModel == 0) {
                mInterface->SendError(this->GetName() + ": could not detect model type");
                CMN_LOG_CLASS_INIT_ERROR << "Startup: Could not detect controller model, "
                                         << "please specify in JSON file" << std::endl;
                // Close connection so we do not hang waiting for data
                Close();
                return;
            }
            mModel = GetModelIndex(autoModel);
            if (mModel < NUM_MODELS) {
                CMN_LOG_CLASS_INIT_VERBOSE << "Startup: setting Galil model to " << autoModel
                                           << " (index = " << mModel << ")" << std::endl;
            }
            else {
                mInterface->SendError(this->GetName() + ": invalid model type");
                // Close connection so we do not hang waiting for data
                Close();
                return;
            }
        }
        else if ((autoModel != 0) && (GetModelIndex(autoModel) != mModel)) {
            mInterface->SendWarning(this->GetName() + ": controller model mismatch (see log file)");
            CMN_LOG_CLASS_INIT_WARNING << "Startup: detected controller model " << autoModel
                                       << " differs from value specified in JSON file "
                                       << ModelTypes[mModel] << std::endl;
        }
    }

    ret = GRecordRate(mGalil, m_configuration.DR_period_ms);
    if (ret != G_NO_ERROR) {
        CMN_LOG_CLASS_INIT_ERROR << "Galil GRecordRate: error " << ret << " setting rate to "
                                 << m_configuration.DR_period_ms << " ms" << std::endl;
        // Close connection so we do not hang waiting for data
        Close();
    }
}

void mtsGalilController::Run()
{
    GDataRecord gRec;
    GReturn ret;

    // Get the Galil data record (DR) and parse it
    if (mGalil) {
        ret = GRecord(mGalil, &gRec, G_DR);
        if (ret == G_NO_ERROR) {
            // First 4 bytes are header (for most controllers)
            if (HasHeader[mModel])
                mHeader = *reinterpret_cast<uint32_t *>(gRec.byte_array);
            // Controller sample number
            mSampleNum = *reinterpret_cast<uint16_t *>(gRec.byte_array + SampleOffset[mModel]);
            mErrorCode = gRec.byte_array[ErrorCodeOffset[mModel]];
            if (AmpStatusOffset[mModel] >= 0)
                mAmpStatus = *reinterpret_cast<uint32_t *>(gRec.byte_array + AmpStatusOffset[mModel]);
            // Get the axis data
            // Since we currently do not care about the last 3 entries (in AxisDataMax), we
            // just cast to AxisDataMin and handle the different offsets.
            bool isAnyMoving = false;
            bool isAllMotorOn = true;
            bool isAllMotorOff = true;
            for (size_t i = 0; i < mNumAxes; i++) {
                unsigned int galilAxis = mAxisToGalilIndexMap[i];
                AxisDataMin *axisPtr = reinterpret_cast<AxisDataMin *>(gRec.byte_array +
                                                                       AxisDataOffset[mModel] +
                                                                       galilAxis*AxisDataSize[mModel]);
                m_measured_js.Position()[i] = (axisPtr->pos - mEncoderOffset[i])/mEncoderCountsPerUnit[i];
                m_measured_js.Velocity()[i] = axisPtr->vel/mEncoderCountsPerUnit[i];
                m_setpoint_js.Position()[i] = (axisPtr->ref_pos - mEncoderOffset[i])/mEncoderCountsPerUnit[i];
                m_setpoint_js.Effort()[i] = (axisPtr->torque*9.9982)/32767.0;  // See Galil TT command
                mAxisStatus[i] = axisPtr->status;     // See Galil User Manual
                mStopCode[i] = axisPtr->stop_code;    // See Galil SC command
                mSwitches[i] = axisPtr->switches;     // See Galil User Manual
                mAnalogIn[i] = axisPtr->analog_in;
                if (mAxisStatus[i] & StatusMotorMoving)
                    isAnyMoving = true;
                if (mAxisStatus[i] & StatusMotorOff)
                    isAllMotorOn = false;
                else
                    isAllMotorOff = false;
                // Following for mActuatorState
                mActuatorState.Position()[i] = m_measured_js.Position()[i];
                mActuatorState.Velocity()[i] = m_measured_js.Velocity()[i];
                mActuatorState.InMotion()[i] = mAxisStatus[i] & StatusMotorMoving;
                mActuatorState.MotorOff()[i] = mAxisStatus[i] & StatusMotorOff;
                mActuatorState.SoftFwdLimitHit()[i] = (mStopCode[i] == SC_FwdLim);
                mActuatorState.SoftRevLimitHit()[i] = (mStopCode[i] == SC_RevLim);
                // NOTE: FwdLimit, RevLimit and Home are affected by the CN command:
                //   CN -1   (default) --> limit switches are active low (default)
                //   CN ,-1  (default) --> home value is based on input voltage (GND --> 0)
                //   CN ,1             --> home value is inverted input voltage (GND --> 1)
                //
                // In either case ("CN ,-1" or "CN ,1"):
                //   - motor homes in reverse direction when home value is 1
                //   - motor homes in forward direction when home value is 0
                //
                // In a typical setup, the limit switches have pull-up resistors, so the
                // active state is low (CN -1).
                // For the home switch, setting CN -1 is appropriate if the home switch is
                // tied to the (active low) reverse limit.
                mActuatorState.HardFwdLimitHit()[i] = mSwitches[i] & SwitchFwdLimit;
                mActuatorState.HardRevLimitHit()[i] = mSwitches[i] & SwitchRevLimit;
                mActuatorState.HomeSwitchOn()[i]    = mSwitches[i] & SwitchHome;
                if (AxisDataSize[mModel] == ADmax) {
                    mActuatorState.IsHomed()[i] = reinterpret_cast<AxisDataMax *>(axisPtr)->var;
                }
                else {
                    // Probably look at a cached version of IsHomed
                }
            }
            // TODO: check following logic
            mActuatorState.SetEStopON(mAmpStatus & (AmpEloUpper | AmpEloLower));
            // TODO: previous implementation used TIME (i.e., "MG TIME"); do we need that, or
            // is it sufficient to use mSampleNum, perhaps scaled by the DR period
            mActuatorState.SetTimestamp(mSampleNum);

            if (!isAllMotorOn && !isAllMotorOff) {
                // If a mix of on/off motors, turn them all off
                mInterface->SendWarning(this->GetName() + ": inconsistent motor power (turning off)");
                DisableMotorPower();
                isAllMotorOn = false;
                isAllMotorOff = true;
            }
            mMotionActive = isAnyMoving;
            mMotorPowerOn = isAllMotorOn;
            m_op_state.SetState(mMotorPowerOn ? prmOperatingState::ENABLED : prmOperatingState::DISABLED);
            m_op_state.SetIsBusy(mMotionActive);
        }
        else {
            mMotionActive = false;
            mMotorPowerOn = false;
            m_op_state.SetState(prmOperatingState::FAULT);
            m_op_state.SetIsBusy(false);
            char buf[128];
            sprintf(buf, ": GRecord error %d", ret);
            mInterface->SendError(this->GetName() + buf);
        }
    }

    // Advance the state table now, so that any connected components can get
    // the latest data.
    StateTable.Advance();

    // Call any connected components
    RunEvent();

    ProcessQueuedCommands();

    switch (mState) {

    case ST_IDLE:
        break;

    case ST_HOMING:
        if (mStopCode.Equal(SC_Homing)) {
            SetHomePosition(mHomePos);
            if (!galil_cmd_common("home (LD-restore)", "LD ", mLimitDisable))
               mInterface->SendError("Home: failed to restore limits");
            mInterface->SendStatus(this->GetName() + ": finished homing");
            mState = ST_IDLE;
        }
        break;
    }
}

void mtsGalilController::Cleanup(){
    Close();
}

// Returns command, followed by list of axes (e.g., "BG ABC")
char *mtsGalilController::WriteCmdAxes(char *buf, const char *cmd, const char *axes)
{
    strcpy(buf, cmd);
    strcat(buf, axes);
    return buf;
}

// Returns command, followed by list of values (e.g., "SP 1000,,500")
char *mtsGalilController::WriteCmdValues(char *buf, const char *cmd, const int32_t *data, const bool *valid, unsigned int num)
{
    strcpy(buf, cmd);
    size_t len = strlen(buf);
    for (unsigned int i = 0; i < num; i++) {
        if (valid[i]) {
            sprintf(buf+len, "%ld,", data[i]);
            len = strlen(buf);
        }
        else {
            buf[len++] = ',';
            buf[len] = 0;
        }
    }
    // Remove last comma
    buf[len-1] = 0;

    return buf;
}

// Issue a query command (e.g., LD ?,?,?) and return the result in the data vector
bool mtsGalilController::QueryCmdValues(const char *cmd, const char *query, vctIntVec &data) const
{
    char sendBuffer[G_SMALL_BUFFER];
    char recvBuffer[G_SMALL_BUFFER];
    strcpy(sendBuffer, cmd);
    strcat(sendBuffer, query);

    GReturn ret = GCmdT(mGalil, sendBuffer, recvBuffer, G_SMALL_BUFFER, 0);
    if (ret == G_NO_ERROR) {
        char *p = recvBuffer;
        int nChars;
        for (size_t i = 0; i < data.size(); i++) {
            long value;
            if (sscanf(p, "%ld%n", &value, &nChars) != 1) {
                mInterface->SendError(this->GetName() + " QueryCmdValues failed for " + recvBuffer);
                return false;
            }
            data[i] = value;
            p += nChars;
            if (*p == ',') p++;
        }
    }
    return (ret == G_NO_ERROR);
}

void mtsGalilController::SendCommand(const std::string &cmdString)
{
    if (mGalil) {
        GReturn ret = GCmd(mGalil, cmdString.c_str());
        if (ret != G_NO_ERROR) {
            char buf[64];
            sprintf(buf, "SendCommand: error %d sending ", ret);
            mInterface->SendError(std::string(buf)+cmdString);
        }
    }
}

void mtsGalilController::SendCommandRet(const std::string &cmdString, std::string &retString)
{
    if (mGalil) {
        char buffer[G_SMALL_BUFFER];
        char *firstChar;
        GReturn ret = GCmdT(mGalil, cmdString.c_str(), buffer, G_SMALL_BUFFER, &firstChar);
        if (ret == G_NO_ERROR) {
            retString.assign(firstChar);
        }
        else {
            retString.clear();
            char buf[64];
            sprintf(buf, "SendCommandRet: error %d sending ", ret);
            mInterface->SendError(std::string(buf)+cmdString);
        }
    }
}

// Enable motor power
void mtsGalilController::EnableMotorPower(void)
{
    SendCommand(WriteCmdAxes(mBuffer, "SH ", mGalilAxes));
}

// Disable motor power
void mtsGalilController::DisableMotorPower(void)
{
    // Sending both ST and MO does not seem to work. Adding AM
    // in between does not seem to help either.
    if (mMotionActive) {
        SendCommand(WriteCmdAxes(mBuffer, "ST ", mGalilAxes));
        // TEMP: set speed in case previous command was servo_jv
        SetSpeed(mSpeed);
    }
    SendCommand(WriteCmdAxes(mBuffer, "MO ", mGalilAxes));
}

void mtsGalilController::AbortProgram()
{
    SendCommand("AB");
}

void mtsGalilController::AbortMotion()
{
    SendCommand("AB 1");
}

bool mtsGalilController::galil_cmd_common(const char *cmdName, const char *cmdGalil,
                                          const vctDoubleVec &data, bool useOffset)
{
    if (!mGalil)
        return false;

    if (data.size() != mNumAxes) {
        mInterface->SendError(this->GetName() + ": size mismatch in " + std::string(cmdName));
        CMN_LOG_CLASS_RUN_ERROR << cmdName << ": size mismatch (data size = " << data.size()
                                << ", num_axes = " << mNumAxes << ")" << std::endl;
        return false;
    }

    int32_t galilData[GALIL_MAX_AXES];
    size_t i;
    for (i = 0; i < mNumAxes; i++) {
        unsigned int galilIndex = mAxisToGalilIndexMap[i];
        int32_t value = static_cast<int32_t>(std::round(data[i]*mEncoderCountsPerUnit[i]));
        if (useOffset)
            value += mEncoderOffset[i];
        galilData[galilIndex] = value;
    }

    SendCommand(WriteCmdValues(mBuffer, cmdGalil, galilData, mGalilIndexValid, mGalilIndexMax));
    return true;
}

bool mtsGalilController::galil_cmd_common(const char *cmdName, const char *cmdGalil,
                                          const vctIntVec &data)
{
    if (!mGalil)
        return false;

    if (data.size() != mNumAxes) {
        mInterface->SendError(this->GetName() + ": size mismatch in " + std::string(cmdName));
        CMN_LOG_CLASS_RUN_ERROR << cmdName << ": size mismatch (data size = " << data.size()
                                << ", num_axes = " << mNumAxes << ")" << std::endl;
        return false;
    }

    int32_t galilData[GALIL_MAX_AXES];
    size_t i;
    for (i = 0; i < mNumAxes; i++) {
        unsigned int galilIndex = mAxisToGalilIndexMap[i];
        galilData[galilIndex] = data[i];
    }

    SendCommand(WriteCmdValues(mBuffer, cmdGalil, galilData, mGalilIndexValid, mGalilIndexMax));
    return true;
}

void mtsGalilController::servo_jp(const prmPositionJointSet &jtpos)
{
    if (!mMotorPowerOn) {
        mInterface->SendError("servo_jp: motor power is off");
        return;
    }
    // Stop motion if active
    if (mMotionActive)
        SendCommand(WriteCmdAxes(mBuffer, "ST ", mGalilAxes));
    if (galil_cmd_common("servo_jp", "PA ", jtpos.Goal(), true))
        SendCommand(WriteCmdAxes(mBuffer, "BG ", mGalilAxes));
}

void mtsGalilController::servo_jr(const prmPositionJointSet &jtpos)
{
    if (!mMotorPowerOn) {
        mInterface->SendError("servo_jr: motor power is off");
        return;
    }
    // Stop motion if active
    if (mMotionActive)
        SendCommand(WriteCmdAxes(mBuffer, "ST ", mGalilAxes));
    if (galil_cmd_common("servo_jr", "PR ", jtpos.Goal(), false))
        SendCommand(WriteCmdAxes(mBuffer, "BG ", mGalilAxes));
}

void mtsGalilController::servo_jv(const prmVelocityJointSet &jtvel)
{
    if (!mMotorPowerOn) {
        mInterface->SendError("servo_jv: motor power is off");
        return;
    }
    // TODO: Only need to send BG after the first JG command
    // Note that JG actually updates SP on the Galil, but for now we do not update
    // mSpeed -- that allows us to restore the previous speed when we stop.
    if (galil_cmd_common("servo_jv", "JG ", jtvel.Goal(), false))
        SendCommand(WriteCmdAxes(mBuffer, "BG ", mGalilAxes));
}

void mtsGalilController::hold(void)
{
    if (!mMotorPowerOn) {
        mInterface->SendError("hold: motor power is off");
        return;
    }
    SendCommand(WriteCmdAxes(mBuffer, "ST ", mGalilAxes));
    // TEMP: set speed in case previous command was servo_jv
    SetSpeed(mSpeed);
}

void mtsGalilController::SetSpeed(const vctDoubleVec &spd)
{
    if (galil_cmd_common("SetSpeed", "SP ", spd, false))
        mSpeed = spd;
}

void mtsGalilController::SetAccel(const vctDoubleVec &accel)
{
    if (galil_cmd_common("SetAccel", "AC ", accel, false))
        mAccel = accel;
}

void mtsGalilController::SetDecel(const vctDoubleVec &decel)
{
    if (galil_cmd_common("SetDecel", "DC ", decel, false))
        mDecel = decel;
}

const bool *mtsGalilController::GetGalilIndexValid(const vctBoolVec &mask) const
{
    unsigned int i;
    static bool galilIndexValid[GALIL_MAX_AXES];
    for (i = 0; i < mGalilIndexMax; i++)
        galilIndexValid[i] = false;
    for (i = 0; i < mask.size(); i++) {
        if (mask[i]) {
            unsigned int galilIndex = mAxisToGalilIndexMap[i];
            galilIndexValid[galilIndex] = true;
        }
    }
    return galilIndexValid;
}

const char *mtsGalilController::GetGalilAxes(const bool *galilIndexValid) const
{
    static char galilMaskString[GALIL_MAX_AXES+1];
    unsigned int i;
    unsigned int k = 0;
    for (i = 0; i < mGalilIndexMax; i++) {
        if (galilIndexValid[i]) {
            galilMaskString[k++] = 'A' + i;
        }
    }
    galilMaskString[k] = 0;  // NULL terminate
    return galilMaskString;
}

void mtsGalilController::Home(const vctBoolVec &mask)
{
    if (!mMotorPowerOn) {
        mInterface->SendError("Home: motor power is off");
        return;
    }
    const bool *galilIndexValid = GetGalilIndexValid(mask);
    const char *galilAxes = GetGalilAxes(galilIndexValid);

    UnHome(mask);
    if (mMotionActive)
        SendCommand(WriteCmdAxes(mBuffer, "ST ", galilAxes));

    // Check whether limit needs to be disabled
    if (mHomeLimitDisable.Any() && (mHomeLimitDisable != mLimitDisable)) {
        if (!galil_cmd_common("home (LD)", "LD ", mHomeLimitDisable)) {
            mInterface->SendError("Home: failed to disable limits");
            return;
        }
    }

    SendCommand(WriteCmdAxes(mBuffer, "HM ", galilAxes));
    SendCommand(WriteCmdAxes(mBuffer, "BG ", galilAxes));
    mState = ST_HOMING;
}

void mtsGalilController::UnHome(const vctBoolVec &mask)
{
    const bool *galilIndexValid = GetGalilIndexValid(mask);
    int32_t galilData[GALIL_MAX_AXES];
    for (unsigned int i = 0; i < mGalilIndexMax; i++)
        galilData[i] = 0;
    SendCommand(WriteCmdValues(mBuffer, "ZA ", galilData, galilIndexValid, mGalilIndexMax));
}

void mtsGalilController::FindEdge(const vctBoolVec &mask)
{
    if (!mMotorPowerOn) {
        mInterface->SendError("FindEdge: motor power is off");
        return;
    }
    const bool *galilIndexValid = GetGalilIndexValid(mask);
    const char *galilAxes = GetGalilAxes(galilIndexValid);

    if (mMotionActive)
        SendCommand(WriteCmdAxes(mBuffer, "ST ", galilAxes));
    SendCommand(WriteCmdAxes(mBuffer, "FE ", galilAxes));
    SendCommand(WriteCmdAxes(mBuffer, "BG ", galilAxes));
}

void mtsGalilController::FindIndex(const vctBoolVec &mask)
{
    if (!mMotorPowerOn) {
        mInterface->SendError("FindIndex: motor power is off");
        return;
    }
    const bool *galilIndexValid = GetGalilIndexValid(mask);
    const char *galilAxes = GetGalilAxes(galilIndexValid);

    if (mMotionActive)
        SendCommand(WriteCmdAxes(mBuffer, "ST ", galilAxes));
    SendCommand(WriteCmdAxes(mBuffer, "FI ", galilAxes));
    SendCommand(WriteCmdAxes(mBuffer, "BG ", galilAxes));
}

void mtsGalilController::SetHomePosition(const vctDoubleVec &pos)
{
    if (galil_cmd_common("SetHomePosition", "DP ", pos, true)) {
        int32_t galilData[GALIL_MAX_AXES];
        for (unsigned int i = 0; i < mGalilIndexMax; i++)
            galilData[i] = 1;
        SendCommand(WriteCmdValues(mBuffer, "ZA ", galilData, mGalilIndexValid, mGalilIndexMax));
    }
}
