/**
Overtaking Control Header
*/


#ifndef _KALMAN_FILTER_H_
#define _KALMAN_FILTER_H_


#define OID_ADTF_KALMAN_FILTER "adtf.aadc.Kalman"


class Kalman : public adtf::cFilter
{
	ADTF_DECLARE_FILTER_VERSION(OID_ADTF_KALMAN_FILTER, "AADC Kalman", OBJCAT_DataFilter, "AADC Kalman", 1, 0, 0, "Beta Version");

	cInputPin      m_oUSFrontMid;

	cOutputPin	m_oVelocityOut;
	cOutputPin	m_oSteeringAngleOut;
	cOutputPin	m_oManeuerFinished;
	cOutputPin     m_oAcceleration;
	cOutputPin     m_oRelvel;

	cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalRelvel;

	cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalUSFrontLeft;
	cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalUSFrontRight;
	cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalUSFrontMid;	
	cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalUSRight;	
	cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalUSRearRight;	

	cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalLFSteering;
	cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalDistance;

	cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalVelocityOut;
	cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalSteeringAngleOut;
	cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalManeuerFinished;


public:

	Kalman(const tChar* __info);
	virtual ~Kalman();

protected:

	tResult Init(tInitStage eStage, __exception);
	tResult Start(__exception);
	tResult Stop(__exception);
	tResult Shutdown(tInitStage eStage, __exception);
	tResult OnPinEvent(IPin* pSource,
		tInt nEventCode,
		tInt nParam1,
		tInt nParam2,
		IMediaSample* pMediaSample);

	// DDL descriptions
	cObjectPtr<IMediaTypeDescription> m_pCoderDescSignal;
	cObjectPtr<IMediaTypeDescription> m_pCoderDescBool;

	// Parameters for Kalman Filter
	tFloat32 T_step;
	cv::Mat A; // System Matrix
	cv::Mat X_n_nm1; // a priori state estimation
	cv::Mat X_n_n; // kalman state estimation
	cv::Mat P_n_nm1; // a priori covariance estimation
	cv::Mat P_n_n; // kalman covariance estimation
	cv::Mat G; // jacobian of the state equation (df/dx)
	cv::Mat C; // C matrix in the system model (measurement equation)
	cv::Mat C_S; // covariance matrix of the system noise
	cv::Mat C_M; // covariance matrix of the measurement noise
	cv::Mat Gamma; // innovation residual
	cv::Mat S; // innovation covariance
	cv::Mat K; // kalman gain
     cv::Mat Q; // System variance (noise)
	cv::Mat Y; // Gamma
	
	// Variables
	
	// arduino timestamp value
     tUInt32 m_tsArduinoTime;

	tFloat32 VMax;
	tFloat32 VLane;
	tFloat32 SteerLeft;
	tFloat32 SteerRight;
	tFloat32 USRight;
	tFloat32 USRearRight;
	tFloat32 USFrontLeft;
	tFloat32 USFrontRight;
	tFloat32 USFrontMid;
	tFloat32 RelVel;
	tFloat32 Distance[100];
	tFloat32 Distance1;
	tFloat32 Distance2;
	tFloat32 Velocity;
	tFloat32 DistanceSample;
	tFloat32 SteeringAngleOut;
	tFloat32 VelocityOut;

	tInt count;
	
	tFloat32 initialTime;
	tFloat32 timeperiod ;
	tFloat32 currentTime ;
	tFloat32 TTCVal;
	tInt counter;
    	tInt pointer;
	tInt LPFSamples;
    	tFloat32 buffer[10];
    	tFloat32 outputSample;
	//	Methods
	tResult PropertyChanged(const char* strProperty);

	tResult RelativeVelocity(tUInt32 timeStamp);
	tResult CalculateTTC(tUInt32 timeStamp);
	tResult KalmanFiltering(tUInt32 timeStamp,tFloat32 Distance);
	tResult LPFilter(tFloat32);
	
	// Transmitting Lights output
	tResult writeOutputs(tUInt32 timeStamp);
	// Critical Sections
   	
	
	
};

#endif


