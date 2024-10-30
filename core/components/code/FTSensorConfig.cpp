#include "FTSensorConfig.h"

#include <cisstCommon/cmnXMLPath.h>
#include <cisstCommon/cmnPath.h>
#include <cisstCommon/cmnUnits.h>
#include <cisstOSAbstraction.h>

bool ParseFTCalibrationFile(const std::string &file, bool UserAxis){
    
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
        !config.GetXMLValue("/FTSensor/Calibration", "@HWTempComp",     m_Calibration.HWTempCompstr))||
        !config.GetXMLValue("/FTSensor/Calibration", "@GainMultiplier",  m_Calibration.GainMultiplier) ||        
        !config.GetXMLValue("/FTSensor/Calibration", "@CableLossDetection", m_Calibration.CableLossDetectionstr) ||
        !config.GetXMLValue("/FTSensor/Calibration", "@OutputBipolar",  m_Calibration.OutputBipolarstr) 
    {
        return false;
    }

    // Convert some data
    m_Calibration.HWTempComp = ( m_Calibration.HWTempCompstr == "True" )
    m_Calibration.CableLossDetection = ( m_Calibration.CableLossDetectionstr == "True" )
    m_Calibration.OutputBipolar = ( m_Calibration.OutputBipolarstr == "True" )
    m_calibration.BasicTransform.SetSize(m_Calibration.NumGages);
    m_calibration.MaxLoads.SetSize(m_Calibration.NumGages);
    m_calibration.MaxLoads.SetAll(0);
    m_calibration.scale.SetSize(m_Calibration.NumGages);
    m_calibration.name.SetSize(m_Calibration.NumGages);
    m_calibration.BasicMatrix.SetSize(m_Calibration.NumGages,m_Calibration.NumGages);


    // Extracting BasicTransform
    if (!config.GetXMLValue("/FTSensor/Calibration/BasicTransform", "@Dx",    m_calibration.BasicTransform[0]) || 
        !config.GetXMLValue("/FTSensor/Calibration/BasicTransform", "@Dy",    m_calibration.BasicTransform[1]) ||
        !config.GetXMLValue("/FTSensor/Calibration/BasicTransform", "@Dz",    m_calibration.BasicTransform[2]) ||
        !config.GetXMLValue("/FTSensor/Calibration/BasicTransform", "@Rx",    m_calibration.BasicTransform[3]) ||
        !config.GetXMLValue("/FTSensor/Calibration/BasicTransform", "@Ry",    m_calibration.BasicTransform[4]) ||
        !config.GetXMLValue("/FTSensor/Calibration/BasicTransform", "@Rz",    m_calibration.BasicTransform[5]) )
    {
        return false;
    }	

    
    std::string temp_values;

    if (UserAxis){
        // Extracting calibration matrix, name and max
        for (int i=0, i<m_calibration.NumGages,i++){
            if (!config.GetXMLValue("/FTSensor/Calibration/UserAxis[" + std::to_string(i) + "]", "@Name", m_Calibration.name[i]) ||
                !config.GetXMLValue("/FTSensor/Calibration/UserAxis[" + std::to_string(i) + "]", "@values", temp_values) ||
                !config.GetXMLValue("/FTSensor/Calibration/UserAxis[" + std::to_string(i) + "]", "@max", m_Calibration.MaxLoads[i])){
                return false;
            }
            else{
                std::istringstream iss(temp_values);
                double value;
                for (int j = 0; j < m_calibration.NumGages; j++) {
                    iss >> value;
                    m_calibration.BasicMatrix[i][j] = value;
                }
            }
        }
    }
    else{
        // Extracting calibration matrix, name, scale and max
        for (int i=0, i < m_calibration.NumGages,i++){
            if (!config.GetXMLValue("/FTSensor/Calibration/Axis[" + std::to_string(i) + "]", "@Name", m_Calibration.name[i]) ||
                !config.GetXMLValue("/FTSensor/Calibration/Axis[" + std::to_string(i) + "]", "@values", temp_values) ||
                !config.GetXMLValue("/FTSensor/Calibration/Axis[" + std::to_string(i) + "]", "@max", m_Calibration.MaxLoads[i])||
                !config.GetXMLValue("/FTSensor/Calibration/Axis[" + std::to_string(i) + "]", "@scale", m_Calibration.scale[i])){
                return false;
            }
            else{
                std::istringstream iss(temp_values);
                double value;
                for (int j = 0; j < m_calibration.NumGages; j++) {
                    iss >> value;
                    m_calibration.BasicMatrix[i][j] = value/m_Calibration.scale[i];
                }
            }
        }
    }
    Calibrated = true;
    return true
}

bool Voltage2FT(const vctDoubleVec voltage, vctDoubleVec &ft) {
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