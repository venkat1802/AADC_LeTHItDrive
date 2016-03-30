/**
Overtaking Control
*/

#include "stdafx.h"
#include "Kalman.h"

#define PI  3.14159265358979323846

using namespace std;
using namespace cv;


ADTF_FILTER_PLUGIN("AADC Kalman", OID_ADTF_KALMAN_FILTER, Kalman);


Kalman::Kalman(const tChar* __info) : cFilter(__info)
{	
	USFrontMid = 0.f;
	Distance1 = 0;
	Distance2 = 0;
	RelVel = -1;
	counter = 0;
	VelocityOut = 90;
	SteeringAngleOut = 90;
	LPFSamples = 10;
	DistanceSample = 0;

	T_step = 0.05;		// every sample, prev T_step = 0.050;
	A = Mat::zeros(2, 2, CV_32F);
     A.at<float>(0,0) = 1;  A.at<float>(0,1) = T_step;	A.at<float>(1,0) = 0; A.at<float>(1,1) = 1;

	X_n_nm1 = Mat::zeros(2, 1, CV_32F);
	P_n_nm1 = Mat::ones(2, 2, CV_32F);

	X_n_n = Mat::zeros(2, 1, CV_32F);
	X_n_n.at<float>(0, 0) = 2; X_n_n.at<float>(1, 0) = RelVel;

	P_n_n = Mat::zeros(2, 2, CV_32F);
	P_n_n.at<float>(0,0) = 100; P_n_n.at<float>(1,1) = 100;

     Q = Mat::zeros(2, 2, CV_32F);	//System or Process noise (variance)
     Q.at<float>(0, 0) = 0.1; Q.at<float>(1, 1) = 0.1;
	
	C = Mat::zeros(2, 2, CV_32F);  //Measurement Matrix
	C.at<float>(0, 0) = 1; C.at<float>(1, 1) = 0;

	C_M = Mat::eye(2, 2, CV_32F)*0.2;		// Measurement noise (variance)
	
	Gamma = Mat::zeros(2, 1, CV_32F);

	S = Mat::zeros(2, 2, CV_32F);
 
	K = Mat::zeros(2, 2, CV_32F);

     Y = Mat::zeros(2, 1, CV_32F);

	//Low pass filter Initialisations 
	for (int i = 0 ; i<10 ; i++) { buffer[i] = 0; }
	pointer = 0;

}

Kalman::~Kalman()
{
}


tResult Kalman::Init(tInitStage eStage, __exception)
{

	RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

	if (eStage == StageFirst)
	{

		cObjectPtr<IMediaDescriptionManager> pDescManager;
		RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**)&pDescManager, __exception_ptr));
		
		//US_front_mid
		tChar const * strDescSignalUSFrontMid = pDescManager->GetMediaDescription("tSignalValue");
		RETURN_IF_POINTER_NULL(strDescSignalUSFrontMid);
		cObjectPtr<IMediaType> pTypeSignalUSFrontMid = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalUSFrontMid, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(pTypeSignalUSFrontMid->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalUSFrontMid));
		RETURN_IF_FAILED(m_oUSFrontMid.Create("US_front_mid", pTypeSignalUSFrontMid, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oUSFrontMid));
			
		// Relative Velocity Out 
		tChar const * strDescSignalRelVelocityOut = pDescManager->GetMediaDescription("tSignalValue");
		RETURN_IF_POINTER_NULL(strDescSignalRelVelocityOut);
		cObjectPtr<IMediaType> pTypeSignalRelVelocityOut = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalRelVelocityOut, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(pTypeSignalRelVelocityOut->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalRelvel));
		RETURN_IF_FAILED(m_oRelvel.Create("RelVelocity", pTypeSignalRelVelocityOut, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oRelvel));

		// Velocity Out 
		tChar const * strDescSignalVelocityOut = pDescManager->GetMediaDescription("tSignalValue");
		RETURN_IF_POINTER_NULL(strDescSignalVelocityOut);
		cObjectPtr<IMediaType> pTypeSignalVelocityOut = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalVelocityOut, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(pTypeSignalVelocityOut->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalVelocityOut));
		RETURN_IF_FAILED(m_oVelocityOut.Create("Distance", pTypeSignalVelocityOut, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oVelocityOut));

		//Steering Angle Out
		tChar const * strDescSignalSteeringAngleOut = pDescManager->GetMediaDescription("tSignalValue");
		RETURN_IF_POINTER_NULL(strDescSignalSteeringAngleOut);
		cObjectPtr<IMediaType> pTypeSignalSteeringAngleOut = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalSteeringAngleOut, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(pTypeSignalSteeringAngleOut->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalSteeringAngleOut));
		RETURN_IF_FAILED(m_oSteeringAngleOut.Create("TTCVal", pTypeSignalSteeringAngleOut, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oSteeringAngleOut));

		
	}

	else if (eStage == StageNormal)
	{

	}
	else if (eStage == StageGraphReady)
	{

	}

	RETURN_NOERROR;
}

tResult Kalman::PropertyChanged(const char* strProperty)
{

	LPFSamples = GetPropertyFloat("LPFSamples");
	RETURN_NOERROR;
}

tResult Kalman::Start(__exception)
{
	return cFilter::Start(__exception_ptr);
}

tResult Kalman::Stop(__exception)
{
	return cFilter::Stop(__exception_ptr);
}



tResult Kalman::Shutdown(tInitStage eStage, __exception)
{

	if (eStage == StageGraphReady)
	{
	}
	else if (eStage == StageNormal)
	{
	}
	else if (eStage == StageFirst)
	{
	}
	return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult Kalman::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{

	RETURN_IF_POINTER_NULL(pMediaSample);
	RETURN_IF_POINTER_NULL(pSource);

	if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
	{

		tUInt32 timeStamp = 0;
		if (pSource == &m_oUSFrontMid) {
			
			cObjectPtr<IMediaCoder> pCoderInput;
			RETURN_IF_FAILED(m_pCoderDescSignalUSFrontMid->Lock(pMediaSample, &pCoderInput));
			pCoderInput->Get("f32Value", (tVoid*)&USFrontMid);
			pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
			m_pCoderDescSignalUSFrontMid->Unlock(pCoderInput);

			RelativeVelocity(timeStamp);
			
			VelocityOut = Distance1;
			SteeringAngleOut = TTCVal;

            	writeOutputs(timeStamp);
		}	
		else
		{
			RETURN_ERROR(ERR_FAILED);
		}

	}
	RETURN_NOERROR;

}

tResult Kalman::RelativeVelocity(tUInt32 timeStamp)
{	
	counter++;
	TTCVal = 0;
    if(USFrontMid<2)
    {         
		KalmanFiltering(timeStamp,USFrontMid);
		// When object is approaching, Vrel is negative (Current distance - last distance)
		DistanceSample = Distance1 - Distance2; 
		
		/***
			Relative velocity calculation Start
		*/
		
		RelVel = 	DistanceSample/0.05;
		
		tInt RelInt = (int) (RelVel*100);
		RelVel = (float) RelInt/100;	// approx RelVel to 2 decimals
		
		if(abs(RelVel) < 0.1 || abs(RelVel) > 2){
			RelVel = 0;
		}
		/***
			Relative velocity calculation End
		*/
		//distance2 is last sample value
		Distance2 = Distance1;

		// Time to collision calculation		
		if(abs(RelVel) > 0.1 && USFrontMid < 1.5){
			TTCVal = abs(Distance1/RelVel);
		}else{
			TTCVal = 0;
		}
		
	}
	else
    {
		RelVel = 0;
	}
	if(counter == 500){
		counter = 0;
	}
	RETURN_NOERROR;
}



//Kalman filtering 
tResult Kalman::KalmanFiltering(tUInt32 timeStamp,tFloat32 Distance) 
{
     Y.at<float>(0, 0) = Distance; Y.at<float>(1, 0) = 0;
	    
	X_n_nm1 = A*X_n_n;	// < - - - - - - - -- - - - - - - - - - - - -- - - - - - - - --  -- - - - -- -  1st Kalman eqt

	P_n_nm1 = (A * P_n_n * A.t()) + Q; // < - - - - - -  - - - - - - - - - - - - - - -  2nd Kalman eqt

	Gamma = Y - C*X_n_nm1; // < - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 3th Kalman eqt

	S = C * (P_n_nm1 * C.t()) + C_M; // < - - - - - - - - - - - -- - - - - - - - - - - - - - - - - - -  4th Kalman eqt

	K = P_n_nm1 * C.t() * S.inv(); // < - - - - - - - - - - - - -- - - - - - - - - - - - - - - - - - -  5th Kalman eqt

	X_n_n = X_n_nm1 + K*Gamma; // < - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 6th Kalman eqt

	P_n_n = (Mat::eye(2, 2, CV_32F) - K*C)*P_n_nm1; // < - - - - -- - - - - - - - - - - - - - - - - - - 7th Kalman eqt

	//LOG_INFO(cString::Format("Kalman Variance matrix :  %f, %f, %f, %f", P_n_n.at<float>(0,0),P_n_n.at<float>(0,1),P_n_n.at<float>(1,0),P_n_n.at<float>(1,1)).GetPtr());
	//LOG_INFO(cString::Format("Kalman Estimate matrix :  %f, %f", X_n_n.at<float>(0,0),X_n_n.at<float>(1,0)).GetPtr());

	Distance1 = X_n_n.at<float>(0,0);

	//LOG_INFO(cString::Format("TTC RelVel Kalmann --> %f", (RelVel)).GetPtr());
  	RETURN_NOERROR;	
}

tResult Kalman::writeOutputs(tUInt32 timeStamp)
{
	//Relative Velocity to Outputpin
	cObjectPtr<IMediaSample> pMediaSampleRelVelocityOut;
	AllocMediaSample((tVoid**)&pMediaSampleRelVelocityOut);

	cObjectPtr<IMediaSerializer> pSerializerRelVelocityOut;
	m_pCoderDescSignalRelvel->GetMediaSampleSerializer(&pSerializerRelVelocityOut);
	tInt nSizeRelVelocityOut = pSerializerRelVelocityOut->GetDeserializedSize();
	pMediaSampleRelVelocityOut->AllocBuffer(nSizeRelVelocityOut);
	cObjectPtr<IMediaCoder> pCoderOutputRelVelocityOut;
	m_pCoderDescSignalRelvel->WriteLock(pMediaSampleRelVelocityOut, &pCoderOutputRelVelocityOut);
	pCoderOutputRelVelocityOut->Set("f32Value", (tVoid*)&RelVel );	//
	pCoderOutputRelVelocityOut->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
	m_pCoderDescSignalRelvel->Unlock(pCoderOutputRelVelocityOut);
	pMediaSampleRelVelocityOut->SetTime(_clock->GetStreamTime());
	m_oRelvel.Transmit(pMediaSampleRelVelocityOut);

	//Velocity
	cObjectPtr<IMediaSample> pMediaSampleVelocityOut;
	AllocMediaSample((tVoid**)&pMediaSampleVelocityOut);

	cObjectPtr<IMediaSerializer> pSerializerVelocityOut;
	m_pCoderDescSignalVelocityOut->GetMediaSampleSerializer(&pSerializerVelocityOut);
	tInt nSizeVelocityOut = pSerializerVelocityOut->GetDeserializedSize();
	pMediaSampleVelocityOut->AllocBuffer(nSizeVelocityOut);
	cObjectPtr<IMediaCoder> pCoderOutputVelocityOut;
	m_pCoderDescSignalVelocityOut->WriteLock(pMediaSampleVelocityOut, &pCoderOutputVelocityOut);
	pCoderOutputVelocityOut->Set("f32Value", (tVoid*)&(VelocityOut));
	pCoderOutputVelocityOut->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
	m_pCoderDescSignalVelocityOut->Unlock(pCoderOutputVelocityOut);
	pMediaSampleVelocityOut->SetTime(_clock->GetStreamTime());
	m_oVelocityOut.Transmit(pMediaSampleVelocityOut);

	//Steering Angle to Outputpin
	cObjectPtr<IMediaSample> pMediaSampleSteeringAngleOut;
	AllocMediaSample((tVoid**)&pMediaSampleSteeringAngleOut);

	cObjectPtr<IMediaSerializer> pSerializerSteeringAngleOut;
	m_pCoderDescSignalSteeringAngleOut->GetMediaSampleSerializer(&pSerializerSteeringAngleOut);
	tInt nSizeSteeringAngleOut = pSerializerSteeringAngleOut->GetDeserializedSize();
	pMediaSampleSteeringAngleOut->AllocBuffer(nSizeSteeringAngleOut);
	cObjectPtr<IMediaCoder> pCoderOutputSteeringAngleOut;
	m_pCoderDescSignalSteeringAngleOut->WriteLock(pMediaSampleSteeringAngleOut, &pCoderOutputSteeringAngleOut);
	pCoderOutputSteeringAngleOut->Set("f32Value", (tVoid*)&(SteeringAngleOut));
	pCoderOutputSteeringAngleOut->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
	m_pCoderDescSignalSteeringAngleOut->Unlock(pCoderOutputSteeringAngleOut);
	pMediaSampleSteeringAngleOut->SetTime(_clock->GetStreamTime());
	m_oSteeringAngleOut.Transmit(pMediaSampleSteeringAngleOut);


	RETURN_NOERROR;

}


tResult Kalman::CalculateTTC(tUInt32 timeStamp){

		// Time to collision calculation
		if(abs(RelVel) > 0.1 && USFrontMid < 1.5){
			TTCVal = abs(Distance1/RelVel);
		}
		//LOG_INFO(cString::Format("TTC Distance --> %f",USFrontMid).GetPtr());
		//LOG_INFO(cString::Format("TTC CalculateTTC --> %f",TTCVal).GetPtr());

		RETURN_NOERROR;
}