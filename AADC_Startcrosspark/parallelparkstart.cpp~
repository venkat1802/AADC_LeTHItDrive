#include "stdafx.h"
#include "parallelparkstart.h"

#define MARKER_ID_UNMARKEDINTERSECTION 0
#define	MARKER_ID_STOPANDGIVEWAY 1
#define	MARKER_ID_PARKINGAREA 2
#define	MARKER_ID_HAVEWAY 3
#define MARKER_ID_AHEADONLY 4
#define	MARKER_ID_GIVEWAY 5
#define	MARKER_ID_PEDESTRIANCROSSING 6
#define MARKER_ID_ROUNDABOUT 7
#define	MARKER_ID_NOOVERTAKING 8
#define	MARKER_ID_NOENTRYVEHICULARTRAFFIC 9
#define	MARKER_ID_ONEWAYSTREET 0


#define PARALLELPARK_SIGNALID_START 15
#define PARALLELPARK_SIGNALID_FINISH 16


// for property browser
//#define MD_REQUIRED_MARKER_SIZE "ManeuverDetection::Required marker size"
#define PP_DISTANCE_ADD1 "initPP::DISTANCE_ADD1"
#define PP_DISTANCE_ADD2 "initPP::DISTANCE_ADD2"
#define PP_DISTANCE_ADD3 "initPP::DISTANCE_ADD3"
#define PP_DISTANCE_ADD4 "initPP::DISTANCE_ADD4"
#define PP_DISTANCE_ADD5 "initPP::DISTANCE_ADD5"
#define PP_DISTANCE_ADD6 "initPP::DISTANCE_ADD6"
#define PP_DISTANCE_ADD7 "initPP::DISTANCE_ADD7"
#define PP_PARK_ACCEL "initPP::PARK_ACCEL"
#define PP_PARK_ACCEL_REV "initPP::PARK_ACCEL_REV"
#define PP_STG3STR "initPP::STG3STR"
#define PP_ENABLE_IMSHOW  "initPP::Enableimshow"
#define MD_REQUIRED_MARKER_SIZE "initPP::Required marker size"

ADTF_FILTER_PLUGIN("Parallel Park Initialize",OID_ADTF_DEMO_START_PARALLEL_PARK, cParallelParkInit)



cParallelParkInit::cParallelParkInit(const tChar* __info):cFilter(__info)
{   


    direct_flag = tFalse;
    
	USRight = 0.f;
	Distnce = 0.f;
	accelin = 90.f;
	Steerin = 90.f;
	yaw = 0.f;
	HazardLight = tFalse;

	
    	SignalID = 0.f;
	//Yval = 0.f;
	Steerout = 0.f;
	accelout = 0.f;
	brakeLights = tFalse; 
	indicatorLeft = tFalse;
	indicatorRight = tFalse;
	headLights = tFalse;	
	
	maneuverstart_flag = tFalse;
	maneuverfinish_flag = tTrue;
	markercheckPassFlag = tFalse;
	Temp_count = 0;
	templateFoundFlag = tFalse;
	found1 = tFalse;
    	foundother = tFalse;
	
	
	currentStage = 0;
	stageStartflag = tFalse;
	stageFinishflag = tFalse;
	//juryinputflag = tFalse;

    	frame_noise = tFalse;
	count1 = 0 ;
	total_count = 0;
	total_count_old = 0; 
	spot = tTrue;

	distance_begin = 0.f;
	distance_desired1 = 0.5f;
	distance_desired2 = 0.5f;
	distance_desired3 = 0.5f;
	distance_desired4 = 0.5f;
	distance_desired5 = 0.5f;
	distance_desired6 = 0.5f;
	distance_desired7 = 0.5f;

	distance_add1 = 0.5f;
	distance_add2 = 0.5f;
	distance_add3 = 0.5f;
	distance_add4 = 0.5f;
	distance_add5 = 0.5f;
	distance_add6 = 0.5f;
	distance_add7 = 0.5f;


	stg3str = 90.f;
	parkaccel = 90.f;
	parkaccelrev = 90.0f;

    SetPropertyBool("Debug Output to Console", m_bDebugModeEnabled);
    SetPropertyStr("Debug Output to Console" NSSUBPROP_DESCRIPTION,"If enabled additional debug information is printed to the console");

    SetPropertyBool(PP_ENABLE_IMSHOW,imshowflag);
    SetPropertyStr(PP_ENABLE_IMSHOW NSSUBPROP_DESCRIPTION, "If true imshow will be enabled");

    SetPropertyFloat(PP_DISTANCE_ADD1, 0.6);
    SetPropertyBool(PP_DISTANCE_ADD1 NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(PP_DISTANCE_ADD1 NSSUBPROP_DESCRIPTION, "stage1 add distance");

    SetPropertyFloat(PP_DISTANCE_ADD2, 0.8);
    SetPropertyBool(PP_DISTANCE_ADD2 NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(PP_DISTANCE_ADD2 NSSUBPROP_DESCRIPTION, "stage2 add distance");

    SetPropertyFloat(PP_DISTANCE_ADD3, 0.75);
    SetPropertyBool(PP_DISTANCE_ADD3 NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(PP_DISTANCE_ADD3 NSSUBPROP_DESCRIPTION, "stage3 add distance");

    SetPropertyFloat(PP_DISTANCE_ADD4, 0.35);
    SetPropertyBool(PP_DISTANCE_ADD4 NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(PP_DISTANCE_ADD4 NSSUBPROP_DESCRIPTION, "stage4 add distance");

    SetPropertyFloat(PP_DISTANCE_ADD5, 0.25);
    SetPropertyBool(PP_DISTANCE_ADD5 NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(PP_DISTANCE_ADD5 NSSUBPROP_DESCRIPTION, "stage5 add distance");

    SetPropertyFloat(PP_DISTANCE_ADD6, 0.7);
    SetPropertyBool(PP_DISTANCE_ADD6 NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(PP_DISTANCE_ADD6 NSSUBPROP_DESCRIPTION, "stage6 add distance");

    SetPropertyFloat(PP_DISTANCE_ADD7, 0.2);
    SetPropertyBool(PP_DISTANCE_ADD7 NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(PP_DISTANCE_ADD7 NSSUBPROP_DESCRIPTION, "stage7 add distance");

    SetPropertyFloat(PP_PARK_ACCEL, 83);
    SetPropertyBool(PP_PARK_ACCEL NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(PP_PARK_ACCEL NSSUBPROP_DESCRIPTION, "parking accel");


    SetPropertyFloat(PP_PARK_ACCEL_REV, 99);
    SetPropertyBool(PP_PARK_ACCEL_REV NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(PP_PARK_ACCEL_REV NSSUBPROP_DESCRIPTION, "parking accel reverse");

    SetPropertyFloat(PP_STG3STR, 77);
    SetPropertyBool(PP_STG3STR NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(PP_STG3STR NSSUBPROP_DESCRIPTION, "stage 3 steering angle");

    SetPropertyFloat(MD_REQUIRED_MARKER_SIZE, 700);
    SetPropertyBool(MD_REQUIRED_MARKER_SIZE NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(MD_REQUIRED_MARKER_SIZE NSSUBPROP_DESCRIPTION, "MarkerDetection::The marker size above which manuevers processing starts");

}

cParallelParkInit::~cParallelParkInit()
{
}

tResult cParallelParkInit::PropertyChanged(const char* strProperty)
{
    ReadProperties(strProperty);
    RETURN_NOERROR;
}

tResult cParallelParkInit::ReadProperties(const tChar* strPropertyName)
{
    
    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, PP_DISTANCE_ADD1))
    {
        distance_add1 = GetPropertyInt(PP_DISTANCE_ADD1);
    }
    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, PP_DISTANCE_ADD2))
    {
        distance_add2 = GetPropertyInt(PP_DISTANCE_ADD2);
    }
    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, PP_DISTANCE_ADD3))
    {
        distance_add3 = GetPropertyInt(PP_DISTANCE_ADD3);
    }
    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, PP_DISTANCE_ADD4))
    {
        distance_add4 = GetPropertyInt(PP_DISTANCE_ADD4);
    }
    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, PP_DISTANCE_ADD5))
    {
        distance_add5 = GetPropertyInt(PP_DISTANCE_ADD5);
    }
    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, PP_DISTANCE_ADD6))
    {
        distance_add6 = GetPropertyInt(PP_DISTANCE_ADD6);
    }
    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, PP_DISTANCE_ADD7))
    {
        distance_add7 = GetPropertyInt(PP_DISTANCE_ADD7);
    }
    /*if (NULL == strPropertyName || cString::IsEqual(strPropertyName, PP_ACCEL_TEST_VAL))
    {
        accel_test_val = GetPropertyInt(PP_ACCEL_TEST_VAL);
    }*/
    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, PP_PARK_ACCEL))
    {
        parkaccel = GetPropertyInt(PP_PARK_ACCEL);
    }
    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, PP_PARK_ACCEL_REV))
    {
        parkaccelrev = GetPropertyInt(PP_PARK_ACCEL_REV);
    }
    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, PP_STG3STR))
    {
        stg3str = GetPropertyInt(PP_STG3STR);
    }

    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, PP_ENABLE_IMSHOW ))
    {
    	imshowflag = GetPropertyBool(PP_ENABLE_IMSHOW);
    }
    if (NULL == strPropertyName || cString::IsEqual(strPropertyName,"Debug Output to Console"))
    {
    	m_bDebugModeEnabled = GetPropertyBool("Debug Output to Console");
    }
    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, MD_REQUIRED_MARKER_SIZE))
    {
    	f32RequiredArea = GetPropertyFloat(MD_REQUIRED_MARKER_SIZE);
    }

    RETURN_NOERROR;

    
}

tResult cParallelParkInit::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));
    if (eStage == StageFirst)
        {

			//create the video rgb input pin
			RETURN_IF_FAILED(m_oVideoInputPin.Create("Video_RGB_input",IPin::PD_Input, static_cast<IPinEventSink*>(this))); 
			RETURN_IF_FAILED(RegisterPin(&m_oVideoInputPin));

			//create the video rgb output pin
			RETURN_IF_FAILED(m_oVideoOutputPin.Create("Video_RGB_output", IPin::PD_Output, static_cast<IPinEventSink*>(this)));
			RETURN_IF_FAILED(RegisterPin(&m_oVideoOutputPin));

			cObjectPtr<IMediaDescriptionManager> pDescManager;
			RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**)&pDescManager, __exception_ptr));

			//Ultrasonic side right
			tChar const * strDescSignalUSRight = pDescManager->GetMediaDescription("tSignalValue");
			RETURN_IF_POINTER_NULL(strDescSignalUSRight);
			cObjectPtr<IMediaType> pTypeSignalUSRight = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalUSRight, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
			RETURN_IF_FAILED(pTypeSignalUSRight->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalUSRight));
			RETURN_IF_FAILED(m_oUSRight.Create("US_Right", pTypeSignalUSRight, static_cast<IPinEventSink*> (this)));
			RETURN_IF_FAILED(RegisterPin(&m_oUSRight));
			
			//LaneFollower Accelaration
			tChar const * strDescSignalLFaccel = pDescManager->GetMediaDescription("tSignalValue");
			RETURN_IF_POINTER_NULL(strDescSignalLFaccel);
			cObjectPtr<IMediaType> pTypeSignalLFaccel = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalLFaccel, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
			RETURN_IF_FAILED(pTypeSignalLFaccel->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalLFaccel));
			RETURN_IF_FAILED(m_oLFaccel.Create("LF_accel", pTypeSignalLFaccel, static_cast<IPinEventSink*> (this)));
			RETURN_IF_FAILED(RegisterPin(&m_oLFaccel));
			
			//Distance

			tChar const * strDescSignalDistance = pDescManager->GetMediaDescription("tSignalValue");
			RETURN_IF_POINTER_NULL(strDescSignalDistance);
			cObjectPtr<IMediaType> pTypeSignalDistance = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalDistance, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
			RETURN_IF_FAILED(pTypeSignalDistance->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalDistance));
			RETURN_IF_FAILED(m_oDistance.Create("Distance", pTypeSignalDistance, static_cast<IPinEventSink*> (this)));
			RETURN_IF_FAILED(RegisterPin(&m_oDistance));



			//Yaw
			//tChar const * strDescSignalYaw = pDescManager->GetMediaDescription("tSignalValue");
			//RETURN_IF_POINTER_NULL(strDescSignalYaw);
			//cObjectPtr<IMediaType> pTypeSignalYaw = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalYaw, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
			//RETURN_IF_FAILED(pTypeSignalYaw->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalYaw));
			//RETURN_IF_FAILED(m_oYaw.Create("Yaw_angle_gyro", pTypeSignalYaw, static_cast<IPinEventSink*> (this)));
			//RETURN_IF_FAILED(RegisterPin(&m_oYaw));
			
			//Lane Follower Steering Angle
			tChar const * strDescSignalLFSteer = pDescManager->GetMediaDescription("tSignalValue");
			RETURN_IF_POINTER_NULL(strDescSignalLFSteer);
			cObjectPtr<IMediaType> pTypeSignalLFSteer = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalLFSteer, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
			RETURN_IF_FAILED(pTypeSignalLFSteer->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalLFsteer));
			RETURN_IF_FAILED(m_oLFsteer.Create("LF_SteeringAngle", pTypeSignalLFSteer, static_cast<IPinEventSink*> (this)));
			RETURN_IF_FAILED(RegisterPin(&m_oLFsteer));
				
			
			//Roadsign
			//cObjectPtr<IMediaTypeDescription> pMediaTypeDesc;
			tChar const * strDesc = pDescManager->GetMediaDescription("tRoadSign");   
			RETURN_IF_POINTER_NULL(strDesc);    
			cObjectPtr<IMediaType> pType = new cMediaType(0, 0, 0, "tRoadSign", strDesc, IMediaDescription::MDF_DDL_DEFAULT_VERSION);        
			// create the road sign OutputPin
			RETURN_IF_FAILED(m_osignDetect.Create("RoadSign", pType, this));
			RETURN_IF_FAILED(RegisterPin(&m_osignDetect));
			// set the description for the road sign pin
			RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionRoadSign));

			

			// output pins actuators
			// accel
			tChar const * strDescSignalAccelOut = pDescManager->GetMediaDescription("tSignalValue");
			RETURN_IF_POINTER_NULL(strDescSignalAccelOut);
			cObjectPtr<IMediaType> pTypeSignalAccelOut = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalAccelOut, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
			RETURN_IF_FAILED(pTypeSignalAccelOut->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescpSignalAccelOut));
			RETURN_IF_FAILED(m_oAccelOut.Create("Acceleration", pTypeSignalAccelOut, static_cast<IPinEventSink*> (this)));
			RETURN_IF_FAILED(RegisterPin(&m_oAccelOut));

			//SignalID goal
			tChar const * strDescSignalSignalID = pDescManager->GetMediaDescription("tSignalValue");
			RETURN_IF_POINTER_NULL(strDescSignalSignalID);
			cObjectPtr<IMediaType> pTypeSignalSignalID = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalSignalID, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
			RETURN_IF_FAILED(pTypeSignalSignalID->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescpSignalSignalID));
			RETURN_IF_FAILED(m_oSignalID.Create("SignalID", pTypeSignalSignalID, static_cast<IPinEventSink*> (this)));
			RETURN_IF_FAILED(RegisterPin(&m_oSignalID));
			
			/*Y goal
			tChar const * strDescSignalYval = pDescManager->GetMediaDescription("tSignalValue");
			RETURN_IF_POINTER_NULL(strDescSignalYval);
			cObjectPtr<IMediaType> pTypeSignalYval = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalYval, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
			RETURN_IF_FAILED(pTypeSignalYval->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescpSignalYval));
			RETURN_IF_FAILED(m_oYval.Create("Yval", pTypeSignalYval, static_cast<IPinEventSink*> (this)));
			RETURN_IF_FAILED(RegisterPin(&m_oYval));*/
			
			// ManeuverFinish
			tChar const * strDescSignalManeuverFinish = pDescManager->GetMediaDescription("tBoolSignalValue");
			RETURN_IF_POINTER_NULL(strDescSignalManeuverFinish);
			cObjectPtr<IMediaType> pTypeSignalManeuverFinish = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalManeuverFinish, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
			RETURN_IF_FAILED(pTypeSignalManeuverFinish->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescpSignalManevuerFinish));
			RETURN_IF_FAILED(m_oManeuverFinish.Create("ManeuverFinish", pTypeSignalManeuverFinish, static_cast<IPinEventSink*> (this)));
			RETURN_IF_FAILED(RegisterPin(&m_oManeuverFinish));
			
			
			//BrakeLights
			tChar const * strDescSignalBrakeLights = pDescManager->GetMediaDescription("tBoolSignalValue");
			RETURN_IF_POINTER_NULL(strDescSignalBrakeLights);
			cObjectPtr<IMediaType> pTypeSignalBrakeLights = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalBrakeLights, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
			RETURN_IF_FAILED(pTypeSignalBrakeLights->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescpSignalBrakeLights));
			RETURN_IF_FAILED(m_oBrakeLights.Create("BrakeLight", pTypeSignalBrakeLights, static_cast<IPinEventSink*> (this)));
			RETURN_IF_FAILED(RegisterPin(&m_oBrakeLights));
			
			//IndicatorLeft
			tChar const * strDescSignalIndicatorLeft = pDescManager->GetMediaDescription("tBoolSignalValue");
			RETURN_IF_POINTER_NULL(strDescSignalIndicatorLeft);
			cObjectPtr<IMediaType> pTypeSignalIndicatorLeft = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalIndicatorLeft, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
			RETURN_IF_FAILED(pTypeSignalIndicatorLeft->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescpSignalIndicatorLeft));
			RETURN_IF_FAILED(m_oIndicatorLeft.Create("Indicator Left", pTypeSignalIndicatorLeft, static_cast<IPinEventSink*> (this)));
			RETURN_IF_FAILED(RegisterPin(&m_oIndicatorLeft));
			
			//IndicatorRight
			tChar const * strDescSignalIndicatorRight = pDescManager->GetMediaDescription("tBoolSignalValue");
			RETURN_IF_POINTER_NULL(strDescSignalIndicatorRight);
			cObjectPtr<IMediaType> pTypeSignalIndicatorRight = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalIndicatorRight, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
			RETURN_IF_FAILED(pTypeSignalIndicatorRight->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescpSignalIndicatorRight));
			RETURN_IF_FAILED(m_oIndicatorRight.Create("Indicator Right", pTypeSignalIndicatorRight, static_cast<IPinEventSink*> (this)));
			RETURN_IF_FAILED(RegisterPin(&m_oIndicatorRight));
			
			//HeadLights
			tChar const * strDescSignalHeadLights = pDescManager->GetMediaDescription("tBoolSignalValue");
			RETURN_IF_POINTER_NULL(strDescSignalHeadLights);
			cObjectPtr<IMediaType> pTypeSignalHeadLights = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalHeadLights, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
			RETURN_IF_FAILED(pTypeSignalHeadLights->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescpSignalHeadLights));
			RETURN_IF_FAILED(m_oHeadLights.Create("HeadLights", pTypeSignalHeadLights, static_cast<IPinEventSink*> (this)));
			RETURN_IF_FAILED(RegisterPin(&m_oHeadLights));
			
			//steering Output
			tChar const * strDescSignalSteerOut = pDescManager->GetMediaDescription("tSignalValue");
			RETURN_IF_POINTER_NULL(strDescSignalSteerOut);
			cObjectPtr<IMediaType> pTypeSignalSteerOut = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalSteerOut, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
			RETURN_IF_FAILED(pTypeSignalSteerOut->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescpSignalSteerOut));
			RETURN_IF_FAILED(m_oSteerOut.Create("Steering Angle", pTypeSignalSteerOut, static_cast<IPinEventSink*> (this)));
			RETURN_IF_FAILED(RegisterPin(&m_oSteerOut));
			
		RETURN_NOERROR;

        }
    else if (eStage == StageNormal)
        { 
		m_bFirstFrame = tTrue;
		i16ID = 0;
		f32Area = 0.0;
       
		ReadProperties(NULL);
        }
    else if (eStage == StageGraphReady)
        {
	m_bIDsRoadSignSet = tFalse;
        // get the image format of the input video pin
        cObjectPtr<IMediaType> pType;
        RETURN_IF_FAILED(m_oVideoInputPin.GetMediaType(&pType));

        cObjectPtr<IMediaTypeVideo> pTypeVideo;
        RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));
    
        // set the image format of the input video pin
        UpdateInputImageFormat(pTypeVideo->GetFormat());  
    
        // set the image format of the output video pin
        UpdateOutputImageFormat(pTypeVideo->GetFormat());  
    

        }
    RETURN_NOERROR;
}

tResult cParallelParkInit::Start(__exception)
{
	return cFilter::Start(__exception_ptr);
}

tResult cParallelParkInit::Stop(__exception)
{	
	imshowflag = tFalse;
	return cFilter::Stop(__exception_ptr);
}

tResult cParallelParkInit::Shutdown(tInitStage eStage, __exception)
{
	if (eStage == StageNormal)
	{
		imshowflag = tFalse;
	}
	return cFilter::Shutdown(eStage, __exception_ptr);
}



tResult cParallelParkInit::OnPinEvent(IPin* pSource,
                                            tInt nEventCode,
                                            tInt nParam1,
                                            tInt nParam2,
                                            IMediaSample* pMediaSample)
{
	RETURN_IF_POINTER_NULL(pMediaSample);
	RETURN_IF_POINTER_NULL(pSource);
    switch (nEventCode)
            {
            case IPinEventSink::PE_MediaSampleReceived:
            		//tUInt32 timeStamp = 0;
            		timeStamp = 0;
            	//other inputs	
				/*if (pSource == &m_ojuryinput) {
					cObjectPtr<IMediaCoder> pCoderInput;
					RETURN_IF_FAILED(m_pCoderDescSignaljuryinput->Lock(pMediaSample, &pCoderInput));
					pCoderInput->Get("bValue", (tVoid*)&maneuverstart_flag);
					pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
					m_pCoderDescSignaljuryinput->Unlock(pCoderInput);
				}*/
				if (pSource == &m_oUSRight) {
					cObjectPtr<IMediaCoder> pCoderInput;
					RETURN_IF_FAILED(m_pCoderDescSignalUSRight->Lock(pMediaSample, &pCoderInput));
					pCoderInput->Get("f32Value", (tVoid*)&USRight);
					pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
					m_pCoderDescSignalUSRight->Unlock(pCoderInput);
				}
				else if (pSource == &m_oLFaccel) {
					cObjectPtr<IMediaCoder> pCoderInput;
					RETURN_IF_FAILED(m_pCoderDescSignalLFaccel->Lock(pMediaSample, &pCoderInput));
					pCoderInput->Get("f32Value", (tVoid*)&accelin);
					pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
					m_pCoderDescSignalLFaccel->Unlock(pCoderInput);
				}            
				else if (pSource == &m_oLFsteer) {
					cObjectPtr<IMediaCoder> pCoderInput;
					RETURN_IF_FAILED(m_pCoderDescSignalLFsteer->Lock(pMediaSample, &pCoderInput));
					pCoderInput->Get("f32Value", (tVoid*)&Steerin);
					pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
					m_pCoderDescSignalLFsteer->Unlock(pCoderInput);
				}
				else if (pSource == &m_oDistance)
				{
					cObjectPtr<IMediaCoder> pCoderInput;
					RETURN_IF_FAILED(m_pCoderDescSignalDistance->Lock(pMediaSample, &pCoderInput));
					pCoderInput->Get("f32Value", (tVoid*)&Distnce);
					pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
					m_pCoderDescSignalDistance->Unlock(pCoderInput);
				}
				/*else if (pSource == &m_oYaw) {
					cObjectPtr<IMediaCoder> pCoderInput;
					RETURN_IF_FAILED(m_pCoderDescSignalYaw->Lock(pMediaSample, &pCoderInput));
					pCoderInput->Get("f32Value", (tVoid*)&yaw);
					pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
					m_pCoderDescSignalYaw->Unlock(pCoderInput);
				}*/
				// a new image was received so the processing is started

				else if (pSource == &m_osignDetect && m_pDescriptionRoadSign != NULL)
				{
					__adtf_sample_read_lock_mediadescription(m_pDescriptionRoadSign, pMediaSample, pCoderInput);
					if (!m_bIDsRoadSignSet)
					{
						pCoderInput->GetID("i16Identifier",m_szIDRoadSignI16Identifier);
						pCoderInput->GetID("f32Imagesize",m_szIDRoadSignF32Imagesize);
						m_bIDsRoadSignSet = tTrue;
					}
					// get the values from sample
					pCoderInput->Get(m_szIDRoadSignI16Identifier, (tVoid*) &i16ID);
					pCoderInput->Get(m_szIDRoadSignF32Imagesize, (tVoid*) &f32Area);

					
					//if (m_bDebugModeEnabled)
					//LOG_INFO(cString::Format("MarkerDetection: Id -> %d Area: %f", i16ID,f32Area));

					if(( MARKER_ID_PARKINGAREA == i16ID ) && (f32Area > f32RequiredArea)) //REQUIRED_AREA)
					{
						maneuverfinish_flag = tFalse;
						markercheckPassFlag = tTrue;
						maneuverstart_flag = tTrue;
						direct_flag = tFalse;
						SignalID = PARALLELPAK_SIGNALID_START;
	
						if (m_bDebugModeEnabled)
						{
							LOG_INFO(cString::Format("marker1 Detection : Id -> %d with required Area: %f is detected", i16ID,f32Area));
						}
					}
				}
				else if (pSource == &m_oVideoInputPin)  // process the video input here
				{
					if(maneuverstart_flag && maneuverfinish_flag)
					{
						maneuverstart_flag = tFalse;
						currentStage = 0;
						SignalID = PARALLELPAK_SIGNALID_FINISH;
					}
					if(markercheckPassFlag && !maneuverfinish_flag)
					{
						accelout = parkaccel;	
						Steerout = Steerin;
						ProcessVideo(pMediaSample);
					}
				}                
				break;

			case IPinEventSink::PE_MediaTypeChanged:
				if (pSource == &m_oVideoInputPin)
				{
					//the input format was changed, so the imageformat has to changed in this filter also
					cObjectPtr<IMediaType> pType;
					RETURN_IF_FAILED(m_oVideoInputPin.GetMediaType(&pType));

					cObjectPtr<IMediaTypeVideo> pTypeVideo;
					RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));

					UpdateInputImageFormat(m_oVideoInputPin.GetFormat());    
					UpdateOutputImageFormat(m_oVideoInputPin.GetFormat());                
				}            
				break;
			default:
                		break;
        }      
    RETURN_NOERROR;
}


tVoid cParallelParkInit::thinningIteration(cv::Mat& img, tInt iter)
{
    CV_Assert(img.channels() == 1);
    CV_Assert(img.depth() != sizeof(uchar));
    CV_Assert(img.rows > 3 && img.cols > 3);

    cv::Mat marker = cv::Mat::zeros(img.size(), CV_8UC1);

    tInt nRows = img.rows;
    tInt nCols = img.cols;

    if (img.isContinuous()) {
        nCols *= nRows;
        nRows = 1;
    }

    tInt x, y;
    uchar *pAbove;
    uchar *pCurr;
    uchar *pBelow;
    uchar *nw, *no, *ne;    // north (pAbove)
    uchar *we, *me, *ea;
    uchar *sw, *so, *se;    // south (pBelow)

    uchar *pDst;

    // initialize row pointers
    pAbove = NULL;
    pCurr  = img.ptr<uchar>(0);
    pBelow = img.ptr<uchar>(1);

    for (y = 1; y < img.rows-1; ++y) {
        // shift the rows up by one
        pAbove = pCurr;
        pCurr  = pBelow;
        pBelow = img.ptr<uchar>(y+1);

        pDst = marker.ptr<uchar>(y);

        // initialize col pointers
        no = &(pAbove[0]);
        ne = &(pAbove[1]);
        me = &(pCurr[0]);
        ea = &(pCurr[1]);
        so = &(pBelow[0]);
        se = &(pBelow[1]);

        for (x = 1; x < img.cols-1; ++x) {
            // shift col pointers left by one (scan left to right)
            nw = no;
            no = ne;
            ne = &(pAbove[x+1]);
            we = me;
            me = ea;
            ea = &(pCurr[x+1]);
            sw = so;
            so = se;
            se = &(pBelow[x+1]);

            int A  = (*no == 0 && *ne == 1) + (*ne == 0 && *ea == 1) + 
                     (*ea == 0 && *se == 1) + (*se == 0 && *so == 1) + 
                     (*so == 0 && *sw == 1) + (*sw == 0 && *we == 1) +
                     (*we == 0 && *nw == 1) + (*nw == 0 && *no == 1);
            int B  = *no + *ne + *ea + *se + *so + *sw + *we + *nw;
            int m1 = iter == 0 ? (*no * *ea * *so) : (*no * *ea * *we);
            int m2 = iter == 0 ? (*ea * *so * *we) : (*no * *so * *we);

            if (A == 1 && (B >= 2 && B <= 6) && m1 == 0 && m2 == 0)
                pDst[x] = 1;
        }
    }

    img &= ~marker;
}

/**
 * Function for thinning the given binary image
 *
 * Parameters:
 * 		src  The source image, binary with range = [0,255]
 * 		dst  The destination image
 */
tVoid cParallelParkInit::thinning(const cv::Mat& src, cv::Mat& dst)
{
    dst = src.clone();
    dst /= 255;         // convert to binary image

    cv::Mat prev = cv::Mat::zeros(dst.size(), CV_8UC1);
    cv::Mat diff;

    do {
        thinningIteration(dst, 0);
        thinningIteration(dst, 1);
        cv::absdiff(dst, prev, diff);
        dst.copyTo(prev);
    } 
    while (cv::countNonZero(diff) > 0);

    dst *= 255;
}

tResult cParallelParkInit::ProcessVideo(adtf::IMediaSample* pISample)
{


    //creating new media sample for output
    cObjectPtr<IMediaSample> pNewSample;
    RETURN_IF_FAILED(_runtime->CreateInstance(OID_ADTF_MEDIA_SAMPLE, IID_ADTF_MEDIA_SAMPLE, (tVoid**) &pNewSample));
    RETURN_IF_FAILED(pNewSample->AllocBuffer(m_sOutputFormat.nSize));    



    //creating new pointer for input data
    const tVoid* l_pSrcBuffer;
    //creating matrix for input image
    Mat TheInputImage;
    Mat TheOutputImage;
    //receiving data from input sample, and saving to TheInputImage
    if (IS_OK(pISample->Lock(&l_pSrcBuffer)))
    {
        //convert to mat
        TheInputImage  = Mat(m_sInputFormat.nHeight,m_sInputFormat.nWidth,CV_8UC3,(tVoid*)l_pSrcBuffer,m_sInputFormat.nBytesPerLine);    
        pISample->Unlock(l_pSrcBuffer); 
		///////////////start

		
		
		
    	if(!direct_flag)
		{
			mytemplate = imread("/home/aadc/AADC/src/aadcUser/src/AADC_Startparallelpark/temp_last.jpg",CV_LOAD_IMAGE_GRAYSCALE);
			img = TheInputImage ;
			img1 = img;
			img = img(Rect(430,275,190,55));
			// Flip the frame horizontally and add blur
			//cv::flip( img, img, 1 );
			cv::cvtColor(img, img, CV_RGB2GRAY);
			GaussianBlur( img, img, Size(7,7), 3.0 );
			cv::threshold(img, img, 0, 255, CV_THRESH_BINARY | THRESH_OTSU) ;
			//LOG_INFO(adtf_util::cString::Format(" %d x %d ", img.rows, img.cols));
			thinning(img, img);
			TheOutputImage =TheInputImage .clone();
			/*img1 = img(Rect(160,100,60,40));	//uncomment to save template
			imwrite("/home/aadc/AADC/src/aadcUser/src/imgprocex1/temp_last.jpg",img1);  */
			//adaptiveThreshold( img, img, 255,ADAPTIVE_THRESH_GAUSSIAN_C,CV_THRESH_BINARY,13, 1 );
			//Mat element1 = getStructuringElement( MORPH_RECT,Size(3,10),Point( 0, 0 ) );
			//Mat element2 = getStructuringElement( MORPH_RECT,Size(10,3),Point( 0, 0 ) );
			//dilate( low1, dst_low, element );
			//dilate( img, img, element1 );
			//erode( img, img, element2 );
			//GaussianBlur( img, img, Size(7,7), 3.0 );

			matchTemplate( img, mytemplate, result, CV_TM_SQDIFF_NORMED );
			normalize( result, result, 0, 1, NORM_MINMAX, -1, Mat() );
		
			//Point match =  minmax( result ); 
			
			minMaxLoc( result, 0, 0, &minLoc, &maxLoc, Mat() );
			matchLoc = minLoc;
			match = matchLoc;
			cv::cvtColor(img, img, CV_GRAY2RGB);
			if((match.x == 0) || (match.y ==0))
			{
				frame_noise = true;
			} 
			else
			{
				frame_noise = false;
		    }
			if(!frame_noise)
			{
				rectangle( img, match, Point( match.x + mytemplate.cols , match.y + mytemplate.rows ), CV_RGB(255, 255, 255), 0.5 );
				//Rect ROI = cv::Rect( match.x, match.y, mytemplate.cols, mytemplate.rows );
		    	if((abs(match.x - match_old.x) > 10) || (abs(match.y - match_old.y) > 10))
				{
					total_count = total_count +1;
					count1 = count1 +1;
					LOG_INFO(adtf_util::cString::Format(" %d ",total_count));
					
					//cout << (total_count_old - total_count )<< "tco" << total_count << " tc " << endl;
				    if((abs(total_count_old-total_count) == 1) && total_count == 1)
					{	found1 = true;
						templateFoundFlag = tTrue;
						currentStage = 1;
						distance_begin = Distnce;
						distance_desired1 = distance_begin + distance_add1;
						LOG_INFO(adtf_util::cString::Format(" %f , %f ",distance_begin, distance_desired1));
					}
					else
					{
						foundother = true;
						LOG_INFO(adtf_util::cString::Format(" foundother , %f ", Distnce ));
					}		
				}
				match_old = match;
				total_count_old = total_count;
			}
     	}
	
		else
		{
			TheOutputImage =TheInputImage.clone();
			templateFoundFlag = tFalse;	
			markercheckPassFlag = tFalse;
		}
		
		if(imshowflag == tTrue)
		{ 
			//imshow("result",result);
			//imshow("img1",img1);
			imshow("mytemplate",mytemplate);
			imshow( "image", img );
			//imshow( "image1", TheInputImage  );
			waitKey(10);
		}
		TheOutputImage =TheInputImage.clone();
		DecisionMaking(timeStamp);
///////////////////end  

    pNewSample->Update(pISample->GetTime(), TheOutputImage.data, m_sOutputFormat.nSize, 0);
    m_oVideoOutputPin.Transmit(pNewSample);  
	 
	cObjectPtr<IMediaSample> pMediaSamplemarkercheckPassFlag;	
	AllocMediaSample((tVoid**)&pMediaSamplemarkercheckPassFlag);
	
    }    

    RETURN_NOERROR;
}



tResult cParallelParkInit::DecisionMaking(tUInt32 timeStamp)
{
	LOG_INFO(adtf_util::cString::Format(" %d ", currentStage ));
	switch (currentStage)
		{	
			case 1:
			{
				if(Distnce < distance_desired1)
				{
					accelout = parkaccel;
				}
				else
				{
					accelout = 90;
					currentStage = 2;
					distance_desired2 = Distnce + distance_add2;
					//maneuverfinish_flag = tTrue;
				}
				//LOG_INFO(adtf_util::cString::Format(" %f ",distance_desired2));
				//DecisionMaking(timeStamp);
				break;
			}

			case 2:
			{
				if(Distnce < distance_desired2)
				{
					accelout = parkaccel;
					if(USRight < .35)
					{
						spot = tFalse;
					}

				}
				else
				{
					if((foundother) && (!spot))
					{
						currentStage = 2;
						distance_desired2 = Distnce + distance_add2;
						LOG_INFO(adtf_util::cString::Format(" %f ",distance_desired2));
						spot = tTrue;
						//writeOutputs(timeStamp);
						//writeFlags(timeStamp);
						foundother = false;
					}
					else
					{
						currentStage = 3;
						accelout = 90;
						distance_desired3 = Distnce + distance_add3;
						//writeOutputs(timeStamp);
						//writeFlags(timeStamp);
						LOG_INFO(adtf_util::cString::Format(" %f ",distance_desired3));
					}		
				}
				break;
			}
			case 3:
			{
				if(Distnce < distance_desired3)
				{
					accelout = parkaccel;
					Steerout = stg3str;
					indicatorLeft = tTrue;
					
				}
				else
				{
					accelout = 90;
					indicatorLeft = tFalse;
					currentStage = 4;
					distance_desired4 = Distnce + distance_add4;
				}
				//writeOutputs(timeStamp);
				//writeFlags(timeStamp);
				break;
			}

			case 4:
			{
				if(Distnce < distance_desired4)
				{
					accelout = parkaccelrev;
					Steerout = 120;
					HazardLight = tTrue;
					//indicatorRight = tTrue;
				}
				else
				{
					accelout = 90;
					currentStage = 5;
					//maneuverfinish_flag = tTrue;
					distance_desired5 = Distnce + distance_add5;
				}
				//writeOutputs(timeStamp);
				//writeFlags(timeStamp);
				break;
			}

			case 5:
			{
				if(Distnce < distance_desired5)
				{
					accelout = parkaccelrev;
					Steerout = 90;
					HazardLight = tTrue;

				}
				else
				{
					accelout = 90;
					currentStage = 6;
					//maneuverfinish_flag = tTrue;
					distance_desired6 = Distnce + distance_add6;
				}
				//writeOutputs(timeStamp);
				//writeFlags(timeStamp);
				break;
			}

			case 6:
			{
				if(Distnce < distance_desired6)
				{
					accelout = parkaccelrev;
					Steerout = 60;
					HazardLight = tTrue;
					
				}
				else
				{
					accelout = 90;
					currentStage = 7;
					distance_desired7 = Distnce + distance_add7;
					//distance_desired = Distnce + 0.65f;
				}
				//writeOutputs(timeStamp);
				//writeFlags(timeStamp);
				break;
			}

			case 7:
			{
				if(Distnce < distance_desired7)
				{
					accelout = parkaccel;
					Steerout = 90;

				}
				else
				{
					accelout = 90;
					Steerout = 90;
					maneuverfinish_flag = tTrue;
					//SignalID = 16;
					//distance_desired5 = Distnce + 0.55;
				}
				//writeOutputs(timeStamp);
				//writeFlags(timeStamp);
				break;
			}


			default:
			{
				break;	
		
			}
		}
		writeOutputs(timeStamp);
		//writeFlags(timeStamp);

	RETURN_NOERROR;
}


tResult cParallelParkInit::UpdateInputImageFormat(const tBitmapFormat* pFormat)
{
    if (pFormat != NULL)
    {
        m_sInputFormat = (*pFormat);
        
        LOG_INFO(adtf_util::cString::Format("Marker Detection Filter: Input: Size %d x %d ; BPL %d ; Size %d , PixelFormat; %d", m_sInputFormat.nWidth,m_sInputFormat.nHeight,m_sInputFormat.nBytesPerLine, m_sInputFormat.nSize, m_sInputFormat.nPixelFormat));        
        
    }
    
    RETURN_NOERROR;
}

tResult cParallelParkInit::UpdateOutputImageFormat(const tBitmapFormat* pFormat)
{
    if (pFormat != NULL)
    {
        m_sOutputFormat = (*pFormat);    
        
        LOG_INFO(adtf_util::cString::Format("Marker Detection Filter: Output: Size %d x %d ; BPL %d ; Size %d, PixelFormat; %d", m_sOutputFormat.nWidth,m_sOutputFormat.nHeight,m_sOutputFormat.nBytesPerLine, m_sOutputFormat.nSize, m_sOutputFormat.nPixelFormat));
        
        m_oVideoOutputPin.SetFormat(&m_sOutputFormat, NULL);                
    }
    
    RETURN_NOERROR;
}

/*tResult cParallelParkInit::writeFlags(tUInt32 timeStamp)
{       
	
	cObjectPtr<IMediaSample> pMediaSamplemarkercheckPassFlag;	
	AllocMediaSample((tVoid**)&pMediaSamplemarkercheckPassFlag);
	
	cObjectPtr<IMediaSerializer> pSerializermarkercheckPassFlag;
	m_pCoderDescpSignalmarkercheckPassFlag->GetMediaSampleSerializer(&pSerializermarkercheckPassFlag);
	tInt nSizemarkercheckPassFlag = pSerializermarkercheckPassFlag->GetDeserializedSize();
	pMediaSamplemarkercheckPassFlag->AllocBuffer(nSizemarkercheckPassFlag);
	cObjectPtr<IMediaCoder> pCoderOutputmarkercheckPassFlag;
	m_pCoderDescpSignalmarkercheckPassFlag->WriteLock(pMediaSamplemarkercheckPassFlag, &pCoderOutputmarkercheckPassFlag);
	pCoderOutputmarkercheckPassFlag->Set("bValue", (tVoid*)&(markercheckPassFlag));
	pCoderOutputmarkercheckPassFlag->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
	m_pCoderDescpSignalmarkercheckPassFlag->Unlock(pCoderOutputmarkercheckPassFlag);
	pMediaSamplemarkercheckPassFlag->SetTime(_clock->GetStreamTime());
	m_omarkerCheckPass.Transmit(pMediaSamplemarkercheckPassFlag);
	
	
	cObjectPtr<IMediaSample> pMediaSampletemplateFoundFlag;	
	AllocMediaSample((tVoid**)&pMediaSampletemplateFoundFlag);
		
	cObjectPtr<IMediaSerializer> pSerializertemplateFoundFlag;
	m_pCoderDescpSignaltemplateFoundFlag->GetMediaSampleSerializer(&pSerializertemplateFoundFlag);
	tInt nSizetemplateFoundFlag = pSerializertemplateFoundFlag->GetDeserializedSize();
	pMediaSampletemplateFoundFlag->AllocBuffer(nSizetemplateFoundFlag);
	cObjectPtr<IMediaCoder> pCoderOutputtemplateFoundFlag;
	m_pCoderDescpSignaltemplateFoundFlag->WriteLock(pMediaSampletemplateFoundFlag, &pCoderOutputtemplateFoundFlag);
	pCoderOutputtemplateFoundFlag->Set("bValue", (tVoid*)&(templateFoundFlag));
	pCoderOutputtemplateFoundFlag->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
	m_pCoderDescpSignaltemplateFoundFlag->Unlock(pCoderOutputtemplateFoundFlag);
	pMediaSampletemplateFoundFlag->SetTime(_clock->GetStreamTime());
	m_otemplateFoundPass.Transmit(pMediaSampletemplateFoundFlag);
	
	
	
	cObjectPtr<IMediaSample> pMediaSampletemplateCount;	
	AllocMediaSample((tVoid**)&pMediaSampletemplateCount);
	
	cObjectPtr<IMediaSerializer> pSerializertemplateCount;
	m_pCoderDescpSignaltemplateCount->GetMediaSampleSerializer(&pSerializertemplateCount);
	tInt nSizetemplateCount = pSerializertemplateCount->GetDeserializedSize();
	pMediaSampletemplateCount->AllocBuffer(nSizetemplateCount);
	cObjectPtr<IMediaCoder> pCoderOutputtemplateCount;
	m_pCoderDescpSignaltemplateCount->WriteLock(pMediaSampletemplateCount, &pCoderOutputtemplateCount);
	pCoderOutputtemplateCount->Set("f32Value", (tVoid*)&(count1));
	pCoderOutputtemplateCount->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
	m_pCoderDescpSignaltemplateCount->Unlock(pCoderOutputtemplateCount);
	pMediaSampletemplateCount->SetTime(_clock->GetStreamTime());
	m_otemplateCount.Transmit(pMediaSampletemplateCount);
	
	RETURN_NOERROR;
}*/

tResult cParallelParkInit::writeLights(tUInt32 timeStamp)
{       

	cObjectPtr<IMediaSample> pMediaSampleBrakeLights;	
	AllocMediaSample((tVoid**)&pMediaSampleBrakeLights);
	
	cObjectPtr<IMediaSerializer> pSerializerBrakeLights;
	m_pCoderDescpSignalBrakeLights->GetMediaSampleSerializer(&pSerializerBrakeLights);
	tInt nSizeBrakeLights = pSerializerBrakeLights->GetDeserializedSize();
	pMediaSampleBrakeLights->AllocBuffer(nSizeBrakeLights);
	cObjectPtr<IMediaCoder> pCoderOutputBrakeLights;
	m_pCoderDescpSignalBrakeLights->WriteLock(pMediaSampleBrakeLights, &pCoderOutputBrakeLights);
	pCoderOutputBrakeLights->Set("bValue", (tVoid*)&(brakeLights));
	pCoderOutputBrakeLights->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
	m_pCoderDescpSignalBrakeLights->Unlock(pCoderOutputBrakeLights);
	pMediaSampleBrakeLights->SetTime(_clock->GetStreamTime());
	m_oBrakeLights.Transmit(pMediaSampleBrakeLights);
	
	
	cObjectPtr<IMediaSample> pMediaSampleIndicatorLeft;	
	AllocMediaSample((tVoid**)&pMediaSampleIndicatorLeft);
	
	cObjectPtr<IMediaSerializer> pSerializerIndicatorLeft;
	m_pCoderDescpSignalIndicatorLeft->GetMediaSampleSerializer(&pSerializerIndicatorLeft);
	tInt nSizeIndicatorLeft = pSerializerIndicatorLeft->GetDeserializedSize();
	pMediaSampleIndicatorLeft->AllocBuffer(nSizeIndicatorLeft);
	cObjectPtr<IMediaCoder> pCoderOutputIndicatorLeft;
	m_pCoderDescpSignalIndicatorLeft->WriteLock(pMediaSampleIndicatorLeft, &pCoderOutputIndicatorLeft);
	pCoderOutputIndicatorLeft->Set("bValue", (tVoid*)&(indicatorLeft));
	pCoderOutputIndicatorLeft->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
	m_pCoderDescpSignalIndicatorLeft->Unlock(pCoderOutputIndicatorLeft);
	pMediaSampleIndicatorLeft->SetTime(_clock->GetStreamTime());
	m_oIndicatorLeft.Transmit(pMediaSampleIndicatorLeft);
	
	
	cObjectPtr<IMediaSample> pMediaSampleIndicatorRight;	
	AllocMediaSample((tVoid**)&pMediaSampleIndicatorRight);
	
	cObjectPtr<IMediaSerializer> pSerializerIndicatorRight;
	m_pCoderDescpSignalIndicatorRight->GetMediaSampleSerializer(&pSerializerIndicatorRight);
	tInt nSizeIndicatorRight = pSerializerIndicatorRight->GetDeserializedSize();
	pMediaSampleIndicatorRight->AllocBuffer(nSizeIndicatorRight);
	cObjectPtr<IMediaCoder> pCoderOutputIndicatorRight;
	m_pCoderDescpSignalIndicatorRight->WriteLock(pMediaSampleIndicatorRight, &pCoderOutputIndicatorRight);
	pCoderOutputIndicatorRight->Set("bValue", (tVoid*)&(indicatorRight));
	pCoderOutputIndicatorRight->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
	m_pCoderDescpSignalIndicatorRight->Unlock(pCoderOutputIndicatorRight);
	pMediaSampleIndicatorRight->SetTime(_clock->GetStreamTime());
	m_oIndicatorRight.Transmit(pMediaSampleIndicatorRight);
	
	
	cObjectPtr<IMediaSample> pMediaSampleHeadLights;	
	AllocMediaSample((tVoid**)&pMediaSampleHeadLights);
	
	cObjectPtr<IMediaSerializer> pSerializerHeadLights;
	m_pCoderDescpSignalHeadLights->GetMediaSampleSerializer(&pSerializerHeadLights);
	tInt nSizeHeadLights = pSerializerHeadLights->GetDeserializedSize();
	pMediaSampleHeadLights->AllocBuffer(nSizeHeadLights);
	cObjectPtr<IMediaCoder> pCoderOutputHeadLights;
	m_pCoderDescpSignalHeadLights->WriteLock(pMediaSampleHeadLights, &pCoderOutputHeadLights);
	pCoderOutputHeadLights->Set("bValue", (tVoid*)&(headLights));
	pCoderOutputHeadLights->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
	m_pCoderDescpSignalHeadLights->Unlock(pCoderOutputHeadLights);
	pMediaSampleHeadLights->SetTime(_clock->GetStreamTime());
	m_oHeadLights.Transmit(pMediaSampleHeadLights);
	
	RETURN_NOERROR;

}


tResult cParallelParkInit::writeOutputs(tUInt32 timeStamp)
{       

	cObjectPtr<IMediaSample> pMediaSampleSignalID;	
	AllocMediaSample((tVoid**)&pMediaSampleSignalID);
	
	cObjectPtr<IMediaSerializer> pSerializerSignalID;
	m_pCoderDescpSignalSignalID->GetMediaSampleSerializer(&pSerializerSignalID);
	tInt nSizeSignalID = pSerializerSignalID->GetDeserializedSize();
	pMediaSampleSignalID->AllocBuffer(nSizeSignalID);

	cObjectPtr<IMediaCoder> pCoderOutputSignalID;
	m_pCoderDescpSignalSignalID->WriteLock(pMediaSampleSignalID, &pCoderOutputSignalID);
	pCoderOutputSignalID->Set("f32Value", (tVoid*)&(SignalID));
	pCoderOutputSignalID->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
	m_pCoderDescpSignalSignalID->Unlock(pCoderOutputSignalID);

	pMediaSampleSignalID->SetTime(_clock->GetStreamTime());
	m_oSignalID.Transmit(pMediaSampleSignalID);




	/*//create new media sample
	cObjectPtr<IMediaSample> pMediaSampleYval;	
	AllocMediaSample((tVoid**)&pMediaSampleYval);
	
	cObjectPtr<IMediaSerializer> pSerializerYval;
	m_pCoderDescpSignalYval->GetMediaSampleSerializer(&pSerializerYval);
	tInt nSizeYval = pSerializerYval->GetDeserializedSize();
	pMediaSampleYval->AllocBuffer(nSizeYval);

	cObjectPtr<IMediaCoder> pCoderOutputYval;
	m_pCoderDescpSignalYval->WriteLock(pMediaSampleYval, &pCoderOutputYval);
	pCoderOutputYval->Set("f32Value", (tVoid*)&(Yval));
	pCoderOutputYval->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
	m_pCoderDescpSignalYval->Unlock(pCoderOutputYval);

	pMediaSampleYval->SetTime(_clock->GetStreamTime());
	m_oYval.Transmit(pMediaSampleYval);*/
	
	//acceleration output
	cObjectPtr<IMediaSample> pMediaSampleAccelOut;
	AllocMediaSample((tVoid**)&pMediaSampleAccelOut);

	cObjectPtr<IMediaSerializer> pSerializerAccelOut;
	m_pCoderDescpSignalAccelOut->GetMediaSampleSerializer(&pSerializerAccelOut);
	tInt nSizeAccelOut = pSerializerAccelOut->GetDeserializedSize();
	pMediaSampleAccelOut->AllocBuffer(nSizeAccelOut);
	cObjectPtr<IMediaCoder> pCoderOutputAccelOut;
	m_pCoderDescpSignalAccelOut->WriteLock(pMediaSampleAccelOut, &pCoderOutputAccelOut);
	pCoderOutputAccelOut->Set("f32Value", (tVoid*)&(accelout));
	pCoderOutputAccelOut->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
	m_pCoderDescpSignalAccelOut->Unlock(pCoderOutputAccelOut);
	pMediaSampleAccelOut->SetTime(_clock->GetStreamTime());
	m_oAccelOut.Transmit(pMediaSampleAccelOut);


	//Transmit Steering Angle to Outputpin
	cObjectPtr<IMediaSample> pMediaSampleSteerOut;
	AllocMediaSample((tVoid**)&pMediaSampleSteerOut);

	cObjectPtr<IMediaSerializer> pSerializerSteerOut;
	m_pCoderDescpSignalSteerOut->GetMediaSampleSerializer(&pSerializerSteerOut);
	tInt nSizeSteerOut = pSerializerSteerOut->GetDeserializedSize();
	pMediaSampleSteerOut->AllocBuffer(nSizeSteerOut);
	cObjectPtr<IMediaCoder> pCoderOutputSteerOut;
	m_pCoderDescpSignalSteerOut->WriteLock(pMediaSampleSteerOut, &pCoderOutputSteerOut);
	pCoderOutputSteerOut->Set("f32Value", (tVoid*)&(Steerout));
	pCoderOutputSteerOut->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
	m_pCoderDescpSignalSteerOut->Unlock(pCoderOutputSteerOut);
	pMediaSampleSteerOut->SetTime(_clock->GetStreamTime());
	m_oSteerOut.Transmit(pMediaSampleSteerOut);


	//Transmit finish maneuver to Outputpin
	cObjectPtr<IMediaSample> pMediaSampleManeuverFinish;
	AllocMediaSample((tVoid**)&pMediaSampleManeuverFinish);

	cObjectPtr<IMediaSerializer> pSerializerManeuverFinish;
	m_pCoderDescpSignalManevuerFinish->GetMediaSampleSerializer(&pSerializerManeuverFinish);
	tInt nSizeFinish = pSerializerManeuverFinish->GetDeserializedSize();
	pMediaSampleManeuverFinish->AllocBuffer(nSizeFinish);
	cObjectPtr<IMediaCoder> pCoderOutputManeuverFinish;
	m_pCoderDescpSignalManevuerFinish->WriteLock(pMediaSampleManeuverFinish, &pCoderOutputManeuverFinish);
	pCoderOutputManeuverFinish->Set("bValue", (tVoid*)&(maneuverfinish_flag));
	pCoderOutputManeuverFinish->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
	m_pCoderDescpSignalManevuerFinish->Unlock(pCoderOutputManeuverFinish);
	pMediaSampleManeuverFinish->SetTime(_clock->GetStreamTime());
	m_oManeuverFinish.Transmit(pMediaSampleManeuverFinish);

	RETURN_NOERROR;

}
