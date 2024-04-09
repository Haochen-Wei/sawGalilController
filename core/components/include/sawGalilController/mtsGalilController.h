/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  (C) Copyright 2024 Johns Hopkins University (JHU), All Rights Reserved.

  This component provides an interface to a Galil DMC controller, using the DR
  (DataRecord) approach, where the Galil controller periodically sends a data
  record. The format of the data record varies based on Galil DMC model type,
  which can be specified (as "galil_model") in the JSON configuration file.
  Valid values of "galil_model" (which is an unsigned integer) are:

      4000   for DMC 4000, 4200, 4103, and 500x0 (default)
     52000   for DMC 52000
      1806   for DMC 1806
      2103   for DMC 2103
      1802   for DMC 1802
     30000   for DMC 30010

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsGalilController_h
#define _mtsGalilController_h

#include <string>

#include <cisstVector/vctDynamicVectorTypes.h>
#include <cisstMultiTask/mtsTaskContinuous.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstParameterTypes/prmConfigurationJoint.h>
#include <cisstParameterTypes/prmStateJoint.h>
#include <cisstParameterTypes/prmPositionJointSet.h>
#include <cisstParameterTypes/prmVelocityJointSet.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmOperatingState.h>
#include <cisstParameterTypes/prmActuatorState.h>

#include <sawGalilController/sawGalilControllerConfig.h>

// Always include last
#include <sawGalilController/sawGalilControllerExport.h>

class CISST_EXPORT mtsGalilController : public mtsTaskContinuous
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_LOD_RUN_ERROR)

 public:

    mtsGalilController(const std::string &name);
    mtsGalilController(const std::string &name, unsigned int sizeStateTable, bool newThread = true);
    mtsGalilController(const mtsTaskContinuousConstructorArg &arg);

    ~mtsGalilController();

    enum { GALIL_MAX_AXES = 8 };

    // cisstMultiTask functions
    void Configure(const std::string &fileName) override;
    void Startup(void) override;
    void Run(void) override;
    void Cleanup(void) override;

protected:

    void         *mGalil;                   // Gcon
    sawGalilControllerConfig::controller m_configuration;

    unsigned int  mModel;                   // Galil model
    unsigned int  mNumAxes;                 // Number of axes
    unsigned int  mGalilIndexMax;           // Maximum galil index
    uint32_t      mHeader;                  // Header bytes in DR packet
    uint16_t      mSampleNum;               // Sample number from controller
    uint8_t       mErrorCode;               // Error code from controller
    uint32_t      mAmpStatus;               // Amplifier status
    prmConfigurationJoint m_config_j;       // Joint configuration
    prmStateJoint m_measured_js;            // Measured joint state (CRTK)
    prmStateJoint m_setpoint_js;            // Setpoint joint state (CRTK)
    prmOperatingState m_op_state;           // Operating state (CRTK)
    prmActuatorState mActuatorState;        // Actuator state
    vctUIntVec    mAxisToGalilIndexMap;     // Map from axis number to Galil index
    vctUIntVec    mGalilIndexToAxisMap;     // Map from Galil index to axis number
    vctDoubleVec  mEncoderCountsPerUnit;    // Encoder conversion factors
    vctLongVec    mEncoderOffset;           // Encoder offset (counts or bits)
    vctDoubleVec  mHomePos;                 // Encoder home positions (offsets)
    vctIntVec     mHomeLimitDisable;        // Limit switch disable during homing
    vctIntVec     mLimitDisable;            // Current setting of limit disable (LD)
    vctUShortVec  mAxisStatus;              // Axis status
    vctUCharVec   mStopCode;                // Axis stop code (see Galil SC command)
    vctUCharVec   mSwitches;                // Axis switches (see Galil TS command)
    vctUShortVec  mAnalogIn;                // Axis analog input
    bool          mMotorPowerOn;            // Whether motor power is on (for all configured motors)
    bool          mMotionActive;            // Whether a motion is active
    vctDoubleVec  mSpeedDefault;            // Default speed
    vctDoubleVec  mSpeed;                   // Current speed
    vctDoubleVec  mAccelDefault;            // Default accel
    vctDoubleVec  mAccel;                   // Current accel
    vctDoubleVec  mDecelDefault;            // Default decel
    vctDoubleVec  mDecel;                   // Current decel
    unsigned int  mState;                   // Internal state machine
    mtsInterfaceProvided *mInterface;       // Provided interface

    // String of configured axes (e.g., "ABC")
    char mGalilAxes[GALIL_MAX_AXES+1];
    // String for querying (e.g., "?,?,?")
    char mGalilQuery[2*GALIL_MAX_AXES];
    // Boolean array indicating which Galil indexes are valid
    bool mGalilIndexValid[GALIL_MAX_AXES];

    char *mBuffer;                          // Local buffer for building command strings

    // Local static method to write cmd and axes to buffer
    // Parameters:
    //    buf    Buffer for output
    //    cmd    Galil command string, including space if desired (e.g, "BG ")
    //    axes   Galil axes string (e.g., "ABC")
    // Example output: "BG ABC"
    static char *WriteCmdAxes(char *buf, const char *cmd, const char *axes);

    // Local method to write a query command (e.g., "LD ?,?,?") and parse the result
    //    cmd    Galil command string to use for query, include space if desired (e.g., "LD ")
    //    query  Query string (e.g., "?,?,?" or "?,,?")
    //    data   Vector for storing result of query
    bool QueryCmdValues(const char *cmd, const char *query, vctIntVec &data) const;

    // Local static method to create cmd followed by comma-separated values
    // Parameters:
    //    buf    Buffer for output
    //    cmd    Galil command string, including space if desired (e.g, "SP ")
    //    data   Data values (indexed by Galil index, so valid values may not be contiguous)
    //    valid  Boolean array indicating which data values are valid
    //    num    Size of data and valid arrays
    // Example output: "SP 1000,,500"
    static char *WriteCmdValues(char *buf, const char *cmd, const int32_t *data, const bool *valid, unsigned int num);

    // Local method to create boolean array from vctBoolVec, also remapping from robot axis to Galil index
    const bool *GetGalilIndexValid(const vctBoolVec &mask) const;
    // Local method to create axes string for specified array of valid Galil indices
    const char *GetGalilAxes(const bool *galilIndexValid) const;

    void Init();
    void Close();

    static unsigned int GetModelIndex(unsigned int modelType);

    void SetupInterfaces();

    void GetNumAxes(unsigned int &numAxes) const { numAxes = mNumAxes; }
    void GetHeader(uint32_t &header) const { header = mHeader; }
    void GetConnected(bool &val) const { val = (mGalil != 0); }

    void SendCommand(const std::string& cmdString);
    void SendCommandRet(const std::string& cmdString, std::string &retString);

    // Enable motor power
    void EnableMotorPower(void);
    // Disable motor power
    void DisableMotorPower(void);

    // Abort robot command
    void AbortProgram();
    void AbortMotion();

    // Common methods for sending command to Galil
    bool galil_cmd_common(const char *cmdName, const char *cmdGalil, const vctDoubleVec &goal,
                          bool useOffset);
    bool galil_cmd_common(const char *cmdName, const char *cmdGalil, const vctIntVec &data);

    // Move joint to specified position
    void servo_jp(const prmPositionJointSet &jtpos);
    // Move joint to specified relative position
    void servo_jr(const prmPositionJointSet &jtpos);
    // Move joint at specified velocity
    void servo_jv(const prmVelocityJointSet &jtvel);
    // Hold joint at current position (Stop)
    void hold(void);

    // Get joint configuration
    void GetConfig_js(prmConfigurationJoint &cfg_j) const
    { cfg_j = m_config_j; }

    // TEMP: following is to be able to use prmStateRobotQtWidgetComponent
    void measured_cp(prmPositionCartesianGet &pos) const
    { pos = prmPositionCartesianGet(); }

    // Set speed, acceleration and deceleration
    void SetSpeed(const vctDoubleVec &spd);
    void SetAccel(const vctDoubleVec &accel);
    void SetDecel(const vctDoubleVec &decel);

    // Home: mask indicates which axes to home
    void Home(const vctBoolVec &mask);
    void UnHome(const vctBoolVec &mask);

    // FindEdge: move specified axes until transition on home input
    void FindEdge(const vctBoolVec &mask);
    // FindIndex: move specified axes until index pulse detected, will set
    // axis position to 0 when done
    void FindIndex(const vctBoolVec &mask);

    // Set absolute position (e.g., for homing); also sets home flag
    // (using ZA) on Galil controller
    void SetHomePosition(const vctDoubleVec &pos);
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsGalilController)

#endif
