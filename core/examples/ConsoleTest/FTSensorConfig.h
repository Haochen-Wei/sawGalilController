#include <cisstVector.h>
#include <string>
#include <sstream>

class FTCalibration: public cmnGenericObject
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION, CMN_LOG_LOD_RUN_ERROR);

public:
	FTCalibration();

	virtual ~FTCalibration();
	
	//This function parses the ATI FT calibration file and fills the FTSensorCalibration structure, returning true on success.
	bool ParseFTCalibrationFile(const std::string &filename,bool UserA);

	//This function converts the voltage readings from the sensor to force/torque readings using the calibration data.
	bool Voltage2FT(const vctDoubleVec voltage, vctDoubleVec &ft);

private:
	typedef struct FTSensorCalibration{
		// basic FT information
		std::string Serial;                        	// serial number of transducer (such as "FT4566")
		std::string BodyStyle;                      // transducer's body style (such as "Delta")
		std::string Family;                         // family of transducer (typ. "DAQ")
		int NumGages;                            	// number of channels
		std::string CalFileVersion;                 // version of calibration file
		
		// calibration information
		std::string PartNumber;                     // calibration part number (such as "US-600-3600")
		std::string CalDate;                      	// date of calibration
		std::string ForceUnits;                   	// force units of basic matrix, as read from file; constant
		std::string TorqueUnits;                  	// torque units of basic matrix, as read from file; constant
		std::string DistUnits;                   	// distance units of basic matrix, as read from file; constant
		std::string OutputMode;                  	// output mode of transducer (such as "Ground Referenced Differential")
		int OutputRange;							// the voltage range of the transducer
		bool HWTempComp;							// whether or not this transducer has hardware temperature compensation.
		std::string HWTempCompstr;					// string representation of HWTempComp
		int GainMultiplier;                         // gain multiplier for the transducer
		bool CableLossDetection;                    // whether or not cable loss detection is enabled
		std::string CableLossDetectionstr;			// string representation of CableLossDetection
		bool OutputBipolar;                         // whether or not the output is bipolar
		std::string OutputBipolarstr;				// string representation of OutputBipolar

		// calibration BasicTransform
		vctDoubleVec BasicTransform;    			// built-in coordinate transform; for internal use

		// calibration matrix
		vctDoubleMat BasicMatrix;					// non-usable matrix; use rt.working_matrix for calculations
		
		// calibration data
		vctDoubleVec MaxLoads;						// maximum loads of each axis, in units above
		vctDoubleVec scale;							// maximum loads of each axis, in units above
		vctDynamicVector<std::string> name;	        // names of each axis
		
	} Calibration;
	
	Calibration m_Calibration;
	bool Calibrated;

};
CMN_DECLARE_SERVICES_INSTANTIATION(FTCalibration);