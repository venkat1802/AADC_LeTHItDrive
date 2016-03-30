/**
 *
 * ADTF Freedemo Project Filter.
 *
 * @file
 * Copyright 
 *
 * $Author: saikiran $
 * $Date: 2016-03-15 16:51:21 +0200 (Mi, 15 Mar 2016) $
 * $Revision: - $
 *
 * @remarks
 *
 */
 
#include "stdafx.h"
#include "freedemo.h"

#define DISTANCE_TRAJ1 "FreeDemo::distance_st1"
#define DISTANCE_TRAJ2 "FreeDemo::distance_st2"
#define DISTANCE_TRAJ3 "FreeDemo::distance_st3"

/// Create filter shell
ADTF_FILTER_PLUGIN("FREE DEMO Filter", OID_ADTF_FREEDEMO_FILTER, FreeDemo);

FreeDemo::FreeDemo(const tChar* __info) :cFilter(__info)
{
	//write values with zero
	USFrontMid = 0;
	N = 0;
	markerCount = 0;
	//for (int i = 0 ; i<1000 ; i++) { buffer[i] = 0; }
	step = 1;
	step_time = 0;
	start_time = 0;
	Start = tTrue;
	Marker = tFalse;
	ManeuerFinished = tTrue;
	distance_st1 = 4;
	DriftVelocity = 60;
	
	LEFTDriftSteer = 60;
	RIGHTDriftSteer = 120;
	
	VelocityOut = 90;
	SteeringAngleOut = 90;
	
	SetPropertyInt("Drifting1", 1);
	SetPropertyBool("Drifting1" NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyStr("Drifting1" NSSUBPROP_DESCRIPTION, "the Step Number to perform from 1 to 6");
	
	SetPropertyFloat(DISTANCE_TRAJ1,2);
	SetPropertyBool(DISTANCE_TRAJ1 NSSUBPROP_ISCHANGEABLE,tTrue);
	SetPropertyStr(DISTANCE_TRAJ1 NSSUBPROP_DESCRIPTION, "the time for the 1st step");

	SetPropertyFloat(DISTANCE_TRAJ2,2);
	SetPropertyBool(DISTANCE_TRAJ2 NSSUBPROP_ISCHANGEABLE,tTrue);
	SetPropertyStr(DISTANCE_TRAJ2 NSSUBPROP_DESCRIPTION, "the time for the 2nd step");
	
	SetPropertyFloat(DISTANCE_TRAJ3,2);
	SetPropertyBool(DISTANCE_TRAJ3 NSSUBPROP_ISCHANGEABLE,tTrue);
	SetPropertyStr(DISTANCE_TRAJ3 NSSUBPROP_DESCRIPTION, "the time for the 3rd step");
	
}

FreeDemo::~FreeDemo()
{

}

tResult FreeDemo::Init(tInitStage eStage, __exception)
{
	// never miss calling the parent implementation!!
	RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

	// in StageFirst you can create and register your static pins.
	if (eStage == StageFirst)
	{
		// create and register the input pin
		//***START***//
		cObjectPtr<IMediaDescriptionManager> pDescManager;
		RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**)&pDescManager, __exception_ptr));
		//-----
		tChar const * strDescSignalValueInput = pDescManager->GetMediaDescription("tSignalValue");
		RETURN_IF_POINTER_NULL(strDescSignalValueInput);
		cObjectPtr<IMediaType> pTypeSignalValueInput = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValueInput, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(pTypeSignalValueInput->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalUSFrontMid));
		//-----	
		RETURN_IF_FAILED(m_oUSFrontMid.Create("USFront", pTypeSignalValueInput, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oUSFrontMid));

		//Roadsign
		//cObjectPtr<IMediaTypeDescription> pMediaTypeDesc;
		tChar const * strDesc = pDescManager->GetMediaDescription("tRoadSign");   
		RETURN_IF_POINTER_NULL(strDesc);    
		cObjectPtr<IMediaType> pType = new cMediaType(0, 0, 0, "tRoadSign", strDesc, IMediaDescription::MDF_DDL_DEFAULT_VERSION);    
		// set the description for the road sign pin
		RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionRoadSign));    
		// create the road sign OutputPin
		RETURN_IF_FAILED(m_oRoadSign.Create("RoadSign1", pType, this));
		RETURN_IF_FAILED(RegisterPin(&m_oRoadSign));
		
		
		//***END***//

		// create and register the Output pins
		tChar const * strDescSignalValueOutput = pDescManager->GetMediaDescription("tSignalValue");
		RETURN_IF_POINTER_NULL(strDescSignalValueOutput);
		cObjectPtr<IMediaType> pTypeSignalValueOutput = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValueOutput, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(pTypeSignalValueOutput->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalOutput));
		RETURN_IF_FAILED(m_oOutputFilter.Create("Output1", pTypeSignalValueOutput, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oOutputFilter));
		
		// Acceleration Out 
		tChar const * strDescSignalVelocityOut = pDescManager->GetMediaDescription("tSignalValue");
		RETURN_IF_POINTER_NULL(strDescSignalVelocityOut);
		cObjectPtr<IMediaType> pTypeSignalVelocityOut = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalVelocityOut, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(pTypeSignalVelocityOut->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalVelocityOut));
		RETURN_IF_FAILED(m_oVelocityOut.Create("Acceleration", pTypeSignalVelocityOut, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oVelocityOut));

		// Steering Angle Out
		tChar const * strDescSignalSteeringAngleOut = pDescManager->GetMediaDescription("tSignalValue");
		RETURN_IF_POINTER_NULL(strDescSignalSteeringAngleOut);
		cObjectPtr<IMediaType> pTypeSignalSteeringAngleOut = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalSteeringAngleOut, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(pTypeSignalSteeringAngleOut->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalSteeringAngleOut));
		RETURN_IF_FAILED(m_oSteeringAngleOut.Create("Steering", pTypeSignalSteeringAngleOut, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oSteeringAngleOut));
		
		// ManeuerFinished Flag Out
		tChar const * strDescSignalManeuerFinished = pDescManager->GetMediaDescription("tBoolSignalValue");
		RETURN_IF_POINTER_NULL(strDescSignalManeuerFinished);
		cObjectPtr<IMediaType> pTypeSignalManeuerFinished = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalManeuerFinished, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(pTypeSignalManeuerFinished->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalManeuerFinished));
		RETURN_IF_FAILED(m_oManeuerFinished.Create("ManeuerFinished", pTypeSignalManeuerFinished, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oManeuerFinished));
		
	}
	else if (eStage == StageNormal)
	{		
        	step = GetPropertyInt("Drifting1");
        	// In this stage you would do further initialisation and/or create your dynamic pins.
		// Please take a look at the demo_dynamicpin example for further reference.
	}
	else if (eStage == StageGraphReady)
	{
		m_bIDsRoadSignSet = tFalse;
		// All pin connections have been established in this stage so you can query your pins
		// about their media types and additional meta data.
		// Please take a look at the demo_imageproc example for further reference.
	}

	RETURN_NOERROR;
}

tResult FreeDemo::Shutdown(tInitStage eStage, __exception)
{
	// In each stage clean up everything that you initiaized in the corresponding stage during Init.
	// Pins are an exception: 
	// - The base class takes care of static pins that are members of this class.
	// - Dynamic pins have to be cleaned up in the ReleasePins method, please see the demo_dynamicpin
	//   example for further reference.

	if (eStage == StageGraphReady)
	{
	}
	else if (eStage == StageNormal)
	{
	}
	else if (eStage == StageFirst)
	{
	}

	// call the base class implementation
	return cFilter::Shutdown(eStage, __exception_ptr);
}


tResult FreeDemo::PropertyChanged(const char* strPropertyName)
{
	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, DISTANCE_TRAJ1)){
		distance_st1 = static_cast<tFloat32> (GetPropertyFloat(DISTANCE_TRAJ1));
	}

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, DISTANCE_TRAJ2)){
		distance_st2 = static_cast<tFloat32> (GetPropertyFloat(DISTANCE_TRAJ2));	}
	
	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, DISTANCE_TRAJ3)){
		distance_st3 = static_cast<tFloat32> (GetPropertyFloat(DISTANCE_TRAJ3));
	}
	
	RETURN_NOERROR;
}
		

tResult FreeDemo::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{

	RETURN_IF_POINTER_NULL(pMediaSample);
	RETURN_IF_POINTER_NULL(pSource);
	// Condition for event MediaSampleReceived
	if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
	{
		tUInt32 timeStamp = 0;		
		// tRoadSign struct input
		if (pSource == &m_oRoadSign && m_pDescriptionRoadSign != NULL)
        	{            
			// read-out the incoming Media Sample m_oRoadSign 
			__adtf_sample_read_lock_mediadescription(m_pDescriptionRoadSign,pMediaSample,pCoderInputSign);       
			
			// get IDs
			if(!m_bIDsRoadSignSet)
			{
				pCoderInputSign->GetID("i16Identifier",m_szIDRoadSignI16Identifier);
				pCoderInputSign->GetID("f32Imagesize", m_szIDRoadSignF32Imagesize);
				m_bIDsRoadSignSet = tTrue;
			}  
			i16ID = 89;
			f32Area = 900;
			// get the values from sample
			pCoderInputSign->Get(m_szIDRoadSignI16Identifier, (tVoid*)&i16ID);
			pCoderInputSign->Get(m_szIDRoadSignF32Imagesize, (tVoid*)&f32Area); 
			

			LOG_INFO(cString::Format("----------Demo Zeichen ID %d erkannt. Area: %f-------------",i16ID,f32Area));

		}
		// Float value input
		else if (pSource == &m_oUSFrontLeft) {
			// read-out the incoming Media Sample
			//get values from media sample
			cObjectPtr<IMediaCoder> pCoderInput;
			RETURN_IF_FAILED(m_pCoderDescSignalUSFrontLeft->Lock(pMediaSample, &pCoderInput));
			pCoderInput->Get("f32Value", (tVoid*)&outputSample);
			pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
			m_pCoderDescSignalUSFrontLeft->Unlock(pCoderInput);
		}
		// Boolean value input
		else if (pSource == &m_oNoOvertake) {
			cObjectPtr<IMediaCoder> pCoderInput;
			RETURN_IF_FAILED(m_pCoderDescSignalNoOvertake->Lock(pMediaSample, &pCoderInput));
			pCoderInput->Get("bValue", (tVoid*)&NoOvertake);
			pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
			m_pCoderDescSignalNoOvertake->Unlock(pCoderInput);
		}
		else if (pSource == &m_oUSFrontMid) {
			cObjectPtr<IMediaCoder> pCoderInput;
			RETURN_IF_FAILED(m_pCoderDescSignalUSFrontMid->Lock(pMediaSample, &pCoderInput));
			pCoderInput->Get("f32Value", (tVoid*)&USFrontMid);
			pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
			m_pCoderDescSignalUSFrontMid->Unlock(pCoderInput);
			
			//if(step > 99)
				//RETURN_NOERROR;
			
			StartAction(timeStamp);
			/*** Emergency STOP - safety check **/
			if (USFrontMid < 0.8 && step > 2) //USFrontLeft < .10 || USFrontRight < .10 || 
				VelocityOut = 90;
			else	if(VelocityOut < 60 || VelocityOut > 120)
				VelocityOut = 90;
			/*** Emergency STOP **/
			
			WriteOutputs(timeStamp);
			
			// Making sure that the car is aligned with the marker.
			if( !Marker && (markerCount < 3 || markerCount > 35) && step > 2 && f32Area > 600 && f32Area < 3500){ 
				
				if(f32Area > 3000)
				{
					step_time = 0.8;		
				}else if(f32Area > 2500)
				{
					step_time = 1.1;			
				}else if(f32Area > 2000)
				{
					step_time = 1.5;			
				}else if(f32Area > 1500)
				{
					step_time = 1.8;			
				}else if(f32Area > 1000)
				{
					step_time = 2.1;			
				}else{
					step_time = 2.25;
				}
				LOG_INFO(cString::Format("---- STEP %d -->Marker Area based Time prediction ------",step));
				LOG_INFO(cString::Format("-------- Marker Area --> %f :: Step_time --> %f -----",f32Area,step_time));
			}
			
			
		}
		else
		{
			RETURN_ERROR(ERR_FAILED);
		}

	}
	RETURN_NOERROR;
}


tResult FreeDemo::StartAction(tUInt32 timeStamp){
		
	if(Start){
		start_time = _clock->GetStreamTime();
		time_diff = (float)(start_time)/1000000; // Converted to Secs
		LOG_INFO(cString::Format("----------Demo : start_time %f-secs------------",time_diff));	
		Start = tFalse;	
	}
	end_time = _clock->GetStreamTime();

	if(end_time < start_time){
		VelocityOut = 90;
		SteeringAngleOut = 90;
		LOG_INFO(cString::Format("-----Demo : Time Reset happened---**STOPPED**-----"));
		RETURN_NOERROR;	
	}	
	
	time_diff = (float)(end_time - start_time)/1000000; // Converted to Secs
	
	if(step == 1){
		/*** Step 1 Dancing - 35 secs**/	
		Action1(timeStamp);
		
	} else if(step == 2) {
		/*** Step 2 STart Drifting - 13 secs**/	
		step_time1 = 5;					// delay
		step_time2 = step_time1 + 0.01;	// Drifting - LEFTDrift
		
		Action2(timeStamp);
		
		/*** Step 2 End **/	
	} else if(step == 3) {
		/*** Step 3 STart Run - 12 secs**/
		step_time1 = 2;					// delay
		step_time2 = step_time1 + 1;		// Go to the Corner towards marker - 1 sec
		step_time3 = step_time2 + 0.5;		// 130 degrees turn
		step_time4 = step_time3 + 2;		//  delay towards center
		step_time5 = step_time4 + 2.5;		// Drifting	- LEFTDrift
		
		Action3(timeStamp);
		
		/*** Step 3 End **/		
	} else if(step == 4) {
		/*** Step 4 STart Run Again - 12 secs**/
		step_time1 = 2;						// delay
		step_time2 = step_time1 + 2;		// Go to the Corner towards marker - 1 sec
		step_time3 = step_time2 + 0.6;		// 180 degrees turn
		step_time4 = step_time3 + 2;		// Move towards center
		step_time5 = step_time4 + 2.5;		// Drifting	LEFTDrift
		
		Action4(timeStamp);
		
		/*** Step 4 End **/		
	} else if(step == 5) {
		/*** Step 5 STart Run Diagonal - 12 secs**/
		step_time1 = 2;						// delay
		step_time2 = step_time1 + 2.2;		// Go to the Corner towards marker - 1 sec
		step_time3 = step_time2 + 0.3;		// 130 degrees left rotation 
		step_time4 = step_time3 + 2;		// delay
		step_time5 = step_time4 + 2.5;		// Drifting	
		
		Action5(timeStamp);
		
		/*** Step 5 End **/		
	}else if(step == 6) {
		/*** Step 6 STart Run - 12 secs**/
		step_time1 = 2;						// delay
		step_time2 = step_time1 + 0.9;		// Go to the Corner towards marker - 1 sec
		step_time3 = step_time2 + 1;		// delay near the center
		step_time4 = step_time3 + 1;		// start drift slow mode
		step_time5 = step_time4 + 6;		// Real Drifting	 
		
		Action6(timeStamp);
		
		/*** Step 6 End **/
	} else {
		ManeuerFinished = tTrue;
		VelocityOut = 90;
		SteeringAngleOut = 90;
		if(time_diff < 1 && step == 999){
			LOG_INFO(cString::Format("----------***Steps finished Successfully***-------------"));	
		}		
	}
	
	if((end_time - start_time) % 25 == 0 && step < 100){
		
		LOG_INFO(cString::Format("---------- :Demo Time_diff %f---------",time_diff));
		//LOG_INFO(cString::Format("---------- :Demo Zeichen ID %d erkannt. Area: %f----------",i16ID,f32Area));
		LOG_INFO(cString::Format("---------- :Demo VelocityOut %f ---&&&--- SteeringAngleOut %f---------",VelocityOut,SteeringAngleOut));	
	}

	//for ( int i = 0; i< N; i++) { }
	RETURN_NOERROR;
}

tResult FreeDemo::Action1(tUInt32 timeStamp){

	/*** Step 1 STart  Dance - 32 secs**/		
		
		step_time1 = 5;
		step_time2 = step_time1 + 3;
		step_time3 = step_time2 + 4;
		step_time4 = step_time3 + 2;		// 14 secs
		step_time5 = step_time4 + 2;	
		step_time6 = step_time5 + 2;
		step_time7 = step_time6 + 2;		// 20 secs
		step_time8 = step_time7 + 2;
		step_time9 = step_time8 + 2; 
		step_time10 = step_time9 + 2; 		// 26 secs
		step_time11 = step_time10 + 2;	
		step_time12 = step_time11 + 3.5;		// 31 secs
		
	/*** Part 1 Start **/
	if(time_diff < step_time4){ 		// 14 secs
		if(time_diff < step_time1){
		
		}else if(time_diff < step_time2){ // 8 secs
			// Move back - 3secs
			VelocityOut = 100;
			SteeringAngleOut = 90;	
		}else if(time_diff < step_time3){ // 12 secs
			// Move front - 4 secs
			VelocityOut = 82;
			SteeringAngleOut = 90;
		}else if(time_diff < step_time4){ // 14 secs
			// Move back - 2 secs
			VelocityOut = 100;
			SteeringAngleOut = 90;
			LOG_INFO(cString::Format("----------Demo : Part 1-------------"));
			/*** Part 1 End **/	
		}	
	}else if(time_diff < step_time7){ 	// for 10 secs
	/*** Part 2 Start Left drive 5 secs **/
		SteeringAngleOut = 61;
		if(time_diff < step_time5){		
			// Stop for a second
			VelocityOut = 90;
		}else if(time_diff < step_time6){
			// Move front - 2 secs
			VelocityOut = 82;
		}else if(time_diff < step_time7){
			// Move Back - 2 secs
			VelocityOut = 98;
			LOG_INFO(cString::Format("----------Demo : Part 2-------------"));
		}
	/*** Part 2 End **/
	}else if(time_diff < step_time10){		// for 10 secs
	/*** Part 3 Start Right drive 5 secs**/
		SteeringAngleOut = 119;
		if(time_diff < step_time8){			// 22 secs
			// Stop for 2 seconds
			VelocityOut = 90;
		}else if(time_diff < step_time9 ){ 	// 24 secs
			// Move Front - 2 secs
			VelocityOut = 82;
		}else if(time_diff < step_time10){	// 26 secs
			// Move Back - 2 secs 
			VelocityOut = 98;
			LOG_INFO(cString::Format("----------Demo : Part 3-------------"));
		}
	/*** Part 3 End (21 secs)**/	
	}else if(time_diff < step_time12){		// for 6 secs	
	/*** Part 4 Start **/	
		if(time_diff < step_time11){
			// Delay - 2 secs
			VelocityOut = 90;
			SteeringAngleOut = 90;
		}else if(time_diff < step_time12){	
			// Steer Back - Take position for next - 3 secs 
			VelocityOut = 98;
			SteeringAngleOut = 120;
			LOG_INFO(cString::Format("----------Demo : Part 4-------------"));
		}
	/*** Part 4 End **/	
	}else{
		// Finish of Maneuer 1, move to next step
		step++;
		//step = 999;
		start_time = end_time;
		time_diff = 0;	
		LOG_INFO(cString::Format("----------Demo : Step Change %d-------------",(step)));
	}
			
	RETURN_NOERROR;
}

tResult FreeDemo::Action2(tUInt32 timeStamp){
	if(time_diff < step_time1){ // 5 secs
		// Ready to Drift - Take position
		VelocityOut = 90;
		SteeringAngleOut = 60;
	}else if(time_diff < step_time2){	
		// Drifting!!! Left steer- 6.5 secs
		VelocityOut = DriftVelocity;
		SteeringAngleOut = LEFTDriftSteer;	// or RIGHTDriftSteer
	}else{
		// Finish of Maneuer 2, move to next step
		time_diff = 0;
		start_time = end_time;
		//step = 999;
		step++;
		Marker = tFalse;
		LOG_INFO(cString::Format("----------Demo : Step Change %d-------------",(step)));
	}
	RETURN_NOERROR;
}

tResult FreeDemo::Action3(tUInt32 timeStamp){
			
	if(!Marker){
		if(time_diff < step_time1){
			// STOP 2 secs - delay
			VelocityOut = 90;
			SteeringAngleOut = 90;
		}else{
			// Revolve slowly to find a marker -- Look around for a marker ID!!
			if(i16ID == 6 && f32Area > 1600)	// STEP 3
			{
				markerCount++;				
			}
		
			if(markerCount > distance_st1 && i16ID == 6)
			{				
				if(markerCount == 5)
					LOG_INFO(cString::Format("--***-STEP %d Greater than 10 markerCount %f ---***---",step,markerCount));
					
				VelocityOut = 90;
				SteeringAngleOut = 90;

				if(markerCount > 20){
					markerCount = 0;
					Marker = tTrue; 
					time_diff = 0;	
					start_time = end_time;
					LOG_INFO(cString::Format("---&&&----Marker TRUE markerCount %f ---&&&---",markerCount));
				}
			}else{
				VelocityOut = 83;
				SteeringAngleOut = LEFTDriftSteer;
			}
		}
	}else{
	
		if(time_diff < step_time1){
			// delay - 2 secs
			VelocityOut = 90;
			SteeringAngleOut = 90;
		}else if(time_diff < step_time2){
			// Go to the Corner - 1 sec
			VelocityOut = 60;
			SteeringAngleOut = 90;
		}else if(time_diff < step_time3){
			// 180 degrees turn at High speed - <0.5 secs
			VelocityOut = DriftVelocity;
			SteeringAngleOut = LEFTDriftSteer;	// or LEFTDriftSteer
		}else if(time_diff < step_time4){
			// Move towards center 2 secs
			VelocityOut = 90;
			SteeringAngleOut = LEFTDriftSteer;
		}else if(time_diff < step_time5){
			// Drifting!!! Left steer - 4 secs
			VelocityOut = DriftVelocity;
			SteeringAngleOut = LEFTDriftSteer;	// or RIGHTDriftSteer
		}else{
			// Finish of Maneuer 3, move to next step
				
			//step++;
			step = 999;
			Marker = tFalse;
			time_diff = 0;	
			start_time = end_time;
			LOG_INFO(cString::Format("----------Demo : Step Change %d-------------",(step)));
		}
	}
	RETURN_NOERROR;
}

tResult FreeDemo::Action4(tUInt32 timeStamp){
			
	if(!Marker){
		if(time_diff < step_time1){
			// STOP 2 secs - delay
			VelocityOut = 90;
			SteeringAngleOut = 90;
		}else{
			// Revolve slowly to find a marker -- Look around for a marker ID!!
			if(i16ID == 5 && f32Area > 900 && f32Area < 1500)	// STEP 4
			{
				markerCount++;				
			}
		
			if(markerCount > distance_st1 && i16ID == 5)
			{				
				if(markerCount == 5)
					LOG_INFO(cString::Format("--***-STEP %d Greater than 10 markerCount %f ---***---",step,markerCount));
				VelocityOut = 90;
				SteeringAngleOut = 90;

				if(markerCount > 15){
					markerCount = 0;
					Marker = tTrue;
					time_diff = 0;	
					start_time = end_time;
					LOG_INFO(cString::Format("---&&&----Marker TRUE markerCount >40 ---&&&---"));
				}
			}else{
				VelocityOut = 83;
				SteeringAngleOut = 118;
			}
		}
	}else{
	
		if(time_diff < step_time1){
			// delay - 3 secs
			VelocityOut = 90;
			SteeringAngleOut = 90;
		}else if(time_diff < step_time2){
			// Go to the Corner - 1.5 secs
			VelocityOut = 60;
			SteeringAngleOut = 90;
		}else if(time_diff < step_time3){
			// 180 degrees turn at High speed - 0.4 secs
			VelocityOut = DriftVelocity;
			SteeringAngleOut = LEFTDriftSteer; //RIGHTDriftSteer
		}else if(time_diff < step_time4){
			// Move front 2 secs
			VelocityOut = 90;
			SteeringAngleOut = 90;
		}else if(time_diff < step_time5){
			// Drifting!!! Left steer - 4 secs
			VelocityOut = DriftVelocity;
			SteeringAngleOut = LEFTDriftSteer;	//RIGHTDriftSteer;
		}else{
			// Finish of Maneuer 3, move to next step
			//step = 999;	
			step++;
			Marker = tFalse;
			time_diff = 0;	
			start_time = end_time;
			LOG_INFO(cString::Format("----------Demo : Step Change %d-------------",(step)));
		}
	}
	RETURN_NOERROR;
}

tResult FreeDemo::Action5(tUInt32 timeStamp){
			
	if(!Marker){
		if(time_diff < step_time1){
			// STOP 2 secs - delay
			VelocityOut = 90;
			SteeringAngleOut = 90;
		}else{
			// Revolve slowly to find a marker -- Look around for a marker ID!!
			if(i16ID == 4 && f32Area > 700 && f32Area < 1400)	// STEP 4
			{
				markerCount++;				
			}
		
			if(markerCount > 10 && i16ID == 9)
			{				
				if(markerCount == 11)
					LOG_INFO(cString::Format("--***-STEP %d Greater than 10 markerCount %f ---***---",step,markerCount));
					
				VelocityOut = 90;
				SteeringAngleOut = 90;

				if(markerCount > 40){
					markerCount = 0;
					Marker = tTrue; 
					time_diff = 0;	
					start_time = end_time;
					LOG_INFO(cString::Format("---&&&----Marker TRUE markerCount %f ---&&&---",markerCount));
				}
			}else{
				VelocityOut = 83;
				SteeringAngleOut = 118;
			}
		}
	}else{
	
		if(time_diff < step_time1 + 3){		// Extra 3 secs ---> 5 secs	& Other car Extra 8 secs ---> 10 secs
			// delay - 3 secs
			VelocityOut = 90;
			SteeringAngleOut = 90;
		}else if(time_diff < step_time2){
			// Go to the Corner - 1.5 secs
			VelocityOut = 60;
			SteeringAngleOut = 90;
		}else if(time_diff < step_time3){
			// 180 degrees turn at High speed - 2 secs
			VelocityOut = DriftVelocity;
			SteeringAngleOut = RIGHTDriftSteer;	// LEFTDriftSteer
		}else if(time_diff < step_time4){
			// delay 2 secs
			VelocityOut = 83;
			SteeringAngleOut = 90;
		}else if(time_diff < step_time5){
			// Drifting!!! Left steer - 3 secs
			VelocityOut = DriftVelocity;
			SteeringAngleOut = RIGHTDriftSteer;	// LEFTDriftSteer
		}else{
			// Finish of Maneuer 3, move to next step
			// step = 999;
			step++;
			Marker = tFalse;
			time_diff = 0;
			start_time = end_time;
			LOG_INFO(cString::Format("----------Demo : Step Change %d-------------",(step)));
		}
	}
	RETURN_NOERROR;
}

tResult FreeDemo::Action6(tUInt32 timeStamp){
			
	if(!Marker){
		if(time_diff < step_time1){
			// STOP 2 secs - delay
			VelocityOut = 90;
			SteeringAngleOut = 90;
		}else{
			// Revolve slowly to find a marker -- Look around for a marker ID!!
			if(i16ID == 3 && f32Area > 700 && f32Area < 1400)	// STEP 6
			{
				markerCount++;				
			}
		
			if(markerCount > 10 && i16ID == 5)
			{				
				if(markerCount == 11)
					LOG_INFO(cString::Format("--***-STEP %d Greater than 10 markerCount %f ---***---",step,markerCount));
				
				VelocityOut = 90;
				SteeringAngleOut = 90;

				if(markerCount > 40){
					markerCount = 0;
					Marker = tTrue; 
					time_diff = 0;
					start_time = end_time;
					LOG_INFO(cString::Format("---&&&----Marker TRUE markerCount %f ---&&&---",markerCount));
				}
			}else{
				VelocityOut = 82;
				SteeringAngleOut = 120;
			}
		}
	}else{	
	
		if(time_diff < step_time1){
			// delay - 2 secs
			VelocityOut = 90;
			SteeringAngleOut = 90;
		}else if(time_diff < step_time2){
			// Go towards Center - only .9 sec
			VelocityOut = 60;
			SteeringAngleOut = 90;
		}else if(time_diff < step_time3){
			// STOP - 1 sec
			VelocityOut = 90;
			SteeringAngleOut = 90;
		}else if(time_diff < step_time4){
			// STart to drift slow mode (1 sec)
			VelocityOut = 83;
			SteeringAngleOut = 75;
		}else if(time_diff < step_time5){
			// Drifting!!! Left steer - 5 secs
			VelocityOut = DriftVelocity;
			SteeringAngleOut = RIGHTDriftSteer;	//LEFTDriftSteer
		}else{
			// Finish of Maneuer 3, move to next step
			// step++;
			step = 999;
			Marker = tFalse;
			time_diff = 0;	
			start_time = end_time;
			LOG_INFO(cString::Format("----------Demo : Step Change %d-------------",(step)));
		}
	}
	RETURN_NOERROR;
}


tResult FreeDemo::WriteOutputs(tUInt32 timeStamp){

	//create new media sample
	cObjectPtr<IMediaSample> pMediaSampleOUT;
	AllocMediaSample((tVoid**)&pMediaSampleOUT);
	//allocate memory with the size given by the descriptor
	cObjectPtr<IMediaSerializer> pSerializer;
	m_pCoderDescSignalOutput->GetMediaSampleSerializer(&pSerializer);
	tInt nSize = pSerializer->GetDeserializedSize();
	pMediaSampleOUT->AllocBuffer(nSize);
	//write date to the media sample with the coder of the descriptor
	cObjectPtr<IMediaCoder> pCoderOutput;
	m_pCoderDescSignalOutput->WriteLock(pMediaSampleOUT, &pCoderOutput);
	// ...
	pCoderOutput->Set("f32Value", (tVoid*)&(outputSample));
	pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
	m_pCoderDescSignalOutput->Unlock(pCoderOutput);
	//transmit media sample over output pin
	pMediaSampleOUT->SetTime(_clock->GetStreamTime());
	m_oOutputFilter.Transmit(pMediaSampleOUT);	
	
	
	//Transmitting Velocity to Outputpin
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

	//Transmitting Steering Angle to Outputpin
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

	//Transmitting Finished to Outputpin
	cObjectPtr<IMediaSample> pMediaSampleFinishedManeuer;
	AllocMediaSample((tVoid**)&pMediaSampleFinishedManeuer);

	cObjectPtr<IMediaSerializer> pSerializerFinishedManeuer;
	m_pCoderDescSignalManeuerFinished->GetMediaSampleSerializer(&pSerializerFinishedManeuer);
	tInt nSizeFinished = pSerializerFinishedManeuer->GetDeserializedSize();
	pMediaSampleFinishedManeuer->AllocBuffer(nSizeFinished);
	cObjectPtr<IMediaCoder> pCoderOutputFinishedManeuer;
	m_pCoderDescSignalManeuerFinished->WriteLock(pMediaSampleFinishedManeuer, &pCoderOutputFinishedManeuer);
	pCoderOutputFinishedManeuer->Set("bValue", (tVoid*)&(ManeuerFinished));
	pCoderOutputFinishedManeuer->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
	m_pCoderDescSignalManeuerFinished->Unlock(pCoderOutputFinishedManeuer);
	pMediaSampleFinishedManeuer->SetTime(_clock->GetStreamTime());
	m_oManeuerFinished.Transmit(pMediaSampleFinishedManeuer);
	
	RETURN_NOERROR;
}

//tResult FreeDemo::StartAction1(tUInt32 timeStamp){RETURN_NOERROR;}
//tResult FreeDemo::StartAction2(tUInt32 timeStamp){RETURN_NOERROR;}


