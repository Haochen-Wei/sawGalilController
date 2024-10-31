#include "FTSensorConfig.h"
#include <cisstCommon/cmnXMLPath.h>
#include <cisstCommon/cmnPath.h>
#include <cisstCommon/cmnUnits.h>
#include <cisstOSAbstraction.h>

CMN_IMPLEMENT_SERVICES(FTCalibration);

FTCalibration::FTCalibration(){
    Calibration m_Calibration;
	Calibrated = false;                        // whether or not the sensor is calibrated
}

FTCalibration::~FTCalibration(){}

bool FTCalibration::ParseFTCalibrationFile(const std::string &file, bool UserAxis){
    // Reading the XML file
    cmnXMLPath config;
    config.SetInputSource(file);
    
    if (// Extracting basic information
        !config.GetXMLValue("/FTSensor", "@Serial",     m_Calibration.Serial) || 
        !config.GetXMLValue("/FTSensor", "@BodyStyle",  m_Calibration.BodyStyle) ||
        !config.GetXMLValue("/FTSensor", "@Family",     m_Calibration.Family) ||
        !config.GetXMLValue("/FTSensor", "@NumGages",   m_Calibration.NumGages) ||
        !config.GetXMLValue("/FTSensor", "@CalFileVersion",     m_Calibration.CalFileVersion) ||
        // Calibration information
        !config.GetXMLValue("/FTSensor/Calibration", "@PartNumber",     m_Calibration.PartNumber) || 
        !config.GetXMLValue("/FTSensor/Calibration", "@CalDate",        m_Calibration.CalDate) ||
        !config.GetXMLValue("/FTSensor/Calibration", "@ForceUnits",     m_Calibration.ForceUnits) ||
        !config.GetXMLValue("/FTSensor/Calibration", "@TorqueUnits",    m_Calibration.TorqueUnits) ||
        !config.GetXMLValue("/FTSensor/Calibration", "@DistUnits",      m_Calibration.DistUnits) ||
        !config.GetXMLValue("/FTSensor/Calibration", "@OutputMode",     m_Calibration.OutputMode) ||
        !config.GetXMLValue("/FTSensor/Calibration", "@OutputRange",    m_Calibration.OutputRange) ||
        !config.GetXMLValue("/FTSensor/Calibration", "@HWTempComp",     m_Calibration.HWTempCompstr)||
        !config.GetXMLValue("/FTSensor/Calibration", "@GainMultiplier",  m_Calibration.GainMultiplier) ||        
        !config.GetXMLValue("/FTSensor/Calibration", "@CableLossDetection", m_Calibration.CableLossDetectionstr) ||
        !config.GetXMLValue("/FTSensor/Calibration", "@OutputBipolar",  m_Calibration.OutputBipolarstr)) 
    {
        return false;
    }
    
    // Convert some data
    m_Calibration.HWTempComp = ( m_Calibration.HWTempCompstr == "True" );
    m_Calibration.CableLossDetection = ( m_Calibration.CableLossDetectionstr == "True" );
    m_Calibration.OutputBipolar = ( m_Calibration.OutputBipolarstr == "True" );
    m_Calibration.BasicTransform.SetSize(m_Calibration.NumGages);
    m_Calibration.MaxLoads.SetSize(m_Calibration.NumGages);
    m_Calibration.MaxLoads.SetAll(0);
    m_Calibration.scale.SetSize(m_Calibration.NumGages);
    m_Calibration.name.SetSize(m_Calibration.NumGages);
    m_Calibration.BasicMatrix.SetSize(m_Calibration.NumGages,m_Calibration.NumGages);


    // Extracting BasicTransform
    if (!config.GetXMLValue("/FTSensor/Calibration/BasicTransform", "@Dx",    m_Calibration.BasicTransform[0]) || 
        !config.GetXMLValue("/FTSensor/Calibration/BasicTransform", "@Dy",    m_Calibration.BasicTransform[1]) ||
        !config.GetXMLValue("/FTSensor/Calibration/BasicTransform", "@Dz",    m_Calibration.BasicTransform[2]) ||
        !config.GetXMLValue("/FTSensor/Calibration/BasicTransform", "@Rx",    m_Calibration.BasicTransform[3]) ||
        !config.GetXMLValue("/FTSensor/Calibration/BasicTransform", "@Ry",    m_Calibration.BasicTransform[4]) ||
        !config.GetXMLValue("/FTSensor/Calibration/BasicTransform", "@Rz",    m_Calibration.BasicTransform[5]) )
    {
        return false;
    }	
    
    
    std::string temp_values;
    int i,j;
    if (UserAxis){
        // Extracting calibration matrix, name and max
        for ( i=0; i < m_Calibration.NumGages ; i++){
            if (!config.GetXMLValue("/FTSensor/Calibration/UserAxis[" + std::to_string(i+1) + "]", "@Name", m_Calibration.name[i]) ||
                !config.GetXMLValue("/FTSensor/Calibration/UserAxis[" + std::to_string(i+1) + "]", "@values", temp_values) ||
                !config.GetXMLValue("/FTSensor/Calibration/UserAxis[" + std::to_string(i+1) + "]", "@max", m_Calibration.MaxLoads[i])){
                return false;
            }
            else{
                std::istringstream iss(temp_values);
                double value;
                for ( j = 0; j < m_Calibration.NumGages; j++) {
                    iss >> value;
                    m_Calibration.BasicMatrix[i][j] = value;
                }
            }
        }
    }
    else{
        // Extracting calibration matrix, name, scale and max
        for ( i=0; i < m_Calibration.NumGages; i++){
            if (!config.GetXMLValue("/FTSensor/Calibration/Axis[" + std::to_string(i+1) + "]", "@Name", m_Calibration.name[i]) ||
                !config.GetXMLValue("/FTSensor/Calibration/Axis[" + std::to_string(i+1) + "]", "@values", temp_values) ||
                !config.GetXMLValue("/FTSensor/Calibration/Axis[" + std::to_string(i+1) + "]", "@max", m_Calibration.MaxLoads[i])||
                !config.GetXMLValue("/FTSensor/Calibration/Axis[" + std::to_string(i+1) + "]", "@scale", m_Calibration.scale[i])){
                return false;
            }
            else{
                std::istringstream iss(temp_values);
                double value;
                for ( j = 0; j < m_Calibration.NumGages; j++) {
                    iss >> value;
                    m_Calibration.BasicMatrix[i][j] = value/m_Calibration.scale[i];
                }
            }
        }
    }
    
    Calibrated = true;
    
    return true;
}

bool FTCalibration::Voltage2FT(const vctDoubleVec voltage, vctDoubleVec &ft) {
    // Convert voltage readings to force/torque data using the calibration data
    // This function will use the calibration matrix and other parameters to perform the conversion
    // The implementation will depend on the specific calibration data structure and conversion logic

    // Add saturation, etc.
    if(!Calibrated){
        return false;
    }
    ft = m_Calibration.BasicMatrix * voltage;
    return true;
}