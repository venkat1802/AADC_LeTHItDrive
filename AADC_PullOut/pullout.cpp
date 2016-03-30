#include "stdafx.h"
//#include <cmath>
#include "pullout.h"

// to speed controller
#define pullout_parallel_active (tFloat32)1.0
#define pulloutleft_active (tFloat32)2.0
#define pulloutright_active (tFloat32)3.0
#define pullout_finished (tFloat32)4.0

// to check the input signal ids
#define START_PULLOVER_LEFT (tFloat32)1.0
#define START_PULLOVER_RIGHT (tFloat32)2.0
#define START_PULLOVER_PARALLEL (tFloat32)3.0
#define PULLOVER_STATUS_DEFAULT (tFloat32)4.0

#define MANEUVER_INTERSECTION_RIGHT      (tFloat32)5.0
#define MANEUVER_INTERSECTION_LEFT       (tFloat32)6.0
#define MANEUVER_INTERSECTION_STRAIGHT   (tFloat32)7.0
#define MANEUVER_INTERSECTION_DEFAULT   (tFloat32)10.0

#define MANEUVER_PARALLEL_PARKING        (tFloat32)14.0
#define MANEUVER_CROSS_PARKING           (tFloat32)15.0
#define MANEUVER_PARKING_DEFAULT         (tFloat32)16.0
#define FLT_EPSILONA (tFloat32)0.01

#define DISTANCE_TRAJ1 "pullout_parallel::distance1"
#define DISTANCE_TRAJ2 "pullout_parallel::distance2"
#define DISTANCE_TRAJ3 "pullout_parallel::distance3"
#define DISTANCE_TRAJ4 "pullout_parallel::distance4"

#define DISTANCE_TRAJ5 "pulloutleft::distance5"
#define DISTANCE_TRAJ6 "pulloutleft::distance6"
#define DISTANCE_TRAJ7 "pulloutleft::distance7"
#define DISTANCE_TRAJ8 "pulloutleft::distance8"

#define DISTANCE_TRAJ9 "pulloutright::distance9"
#define DISTANCE_TRAJ10 "pulloutright::distance10"
#define DISTANCE_TRAJ11 "pulloutright::distance11"



ADTF_FILTER_PLUGIN("AADC Pull Out", OID_ADTF_pullout, cpullout)

cpullout::cpullout(const tChar* __info):cFilter(__info)
{
	distance1 = 0.25;
	Ultrasonic_Front_Left = 0.0f;
	SetPropertyFloat(DISTANCE_TRAJ1,0.25); //0.3 -- count = 0
    	SetPropertyBool(DISTANCE_TRAJ1 NSSUBPROP_ISCHANGEABLE,tTrue);
    	SetPropertyStr(DISTANCE_TRAJ1 NSSUBPROP_DESCRIPTION, "the distance for the 1st trajectory");

	SetPropertyFloat(DISTANCE_TRAJ2, 0.6);
    	SetPropertyBool(DISTANCE_TRAJ2 NSSUBPROP_ISCHANGEABLE,tTrue);
    	SetPropertyStr(DISTANCE_TRAJ2 NSSUBPROP_DESCRIPTION, "the distance for the curve trajectory");

	SetPropertyFloat(DISTANCE_TRAJ3,0.6);
    	SetPropertyBool(DISTANCE_TRAJ3 NSSUBPROP_ISCHANGEABLE,tTrue);
    	SetPropertyStr(DISTANCE_TRAJ3 NSSUBPROP_DESCRIPTION, "the distance for the straight after taking curve trajectory");

	SetPropertyFloat(DISTANCE_TRAJ4,0.2); //0.3 -- count = 0
    	SetPropertyBool(DISTANCE_TRAJ4 NSSUBPROP_ISCHANGEABLE,tTrue);
    	SetPropertyStr(DISTANCE_TRAJ4 NSSUBPROP_DESCRIPTION, "the distance for the 1st trajectory");

	SetPropertyFloat(DISTANCE_TRAJ5,0.25);
    	SetPropertyBool(DISTANCE_TRAJ5 NSSUBPROP_ISCHANGEABLE,tTrue);
    	SetPropertyStr(DISTANCE_TRAJ5 NSSUBPROP_DESCRIPTION, "the distance for the curve trajectory");

	SetPropertyFloat(DISTANCE_TRAJ6,0.9);
    	SetPropertyBool(DISTANCE_TRAJ6 NSSUBPROP_ISCHANGEABLE,tTrue);
    	SetPropertyStr(DISTANCE_TRAJ6 NSSUBPROP_DESCRIPTION, "the distance for the straight after taking curve trajectory");


	SetPropertyFloat(DISTANCE_TRAJ7,0.6); //0.3 -- count = 0
    	SetPropertyBool(DISTANCE_TRAJ7 NSSUBPROP_ISCHANGEABLE,tTrue);
    	SetPropertyStr(DISTANCE_TRAJ7 NSSUBPROP_DESCRIPTION, "the distance for the 1st trajectory");

	SetPropertyFloat(DISTANCE_TRAJ8,0.2);
    	SetPropertyBool(DISTANCE_TRAJ8 NSSUBPROP_ISCHANGEABLE,tTrue);
    	SetPropertyStr(DISTANCE_TRAJ8 NSSUBPROP_DESCRIPTION, "the distance for the curve trajectory");

	SetPropertyFloat(DISTANCE_TRAJ9,0.75);
    	SetPropertyBool(DISTANCE_TRAJ9 NSSUBPROP_ISCHANGEABLE,tTrue);
    	SetPropertyStr(DISTANCE_TRAJ9 NSSUBPROP_DESCRIPTION, "the distance for the straight after taking curve trajectory");

			SetPropertyFloat(DISTANCE_TRAJ10,0.6);
    	SetPropertyBool(DISTANCE_TRAJ10 NSSUBPROP_ISCHANGEABLE,tTrue);
    	SetPropertyStr(DISTANCE_TRAJ10 NSSUBPROP_DESCRIPTION, "the distance for the straight after taking curve trajectory");

			SetPropertyFloat(DISTANCE_TRAJ11,0.2);
    	SetPropertyBool(DISTANCE_TRAJ11 NSSUBPROP_ISCHANGEABLE,tTrue);
    	SetPropertyStr(DISTANCE_TRAJ11 NSSUBPROP_DESCRIPTION, "the distance for the straight after taking curve trajectory");

	active = 5.0;//pullout_finished;
	start= PULLOVER_STATUS_DEFAULT;
	dist_at_start = 0;
    	accelerate = 90.0;
	first_sample= tFalse;
	var_headlight = tFalse;
	var_headlight_left = tFalse;
	flag_switch_left_right = tTrue; // if true right , false left
	timeflag = tFalse;
	//start_time = end_time = _clock->GetStreamTime();
	time_diff = 3000000;
	time_diff_last = 1000000;
	ReadProperties(NULL);
}

cpullout::~cpullout()
{

}
tResult cpullout::Init(tInitStage eStage, __exception)
{

	RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))


		if (eStage == StageFirst)
		{

			// create description manager
			cObjectPtr<IMediaDescriptionManager> pDescManager;
			RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**)&pDescManager, __exception_ptr));

			// get media type for input pins
			tChar const * strDescSignalUltrasonic_Front_Left = pDescManager->GetMediaDescription("tSignalValue");
			RETURN_IF_POINTER_NULL(strDescSignalUltrasonic_Front_Left);
			cObjectPtr<IMediaType> pTypeSignalUltrasonic_Front_Left = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalUltrasonic_Front_Left, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
			RETURN_IF_FAILED(pTypeSignalUltrasonic_Front_Left->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescUltrasonic_Front_Left));
			RETURN_IF_FAILED(m_Ultrasonic_Front_Left.Create("ultrasonic_front_left", pTypeSignalUltrasonic_Front_Left, static_cast<IPinEventSink*> (this)));
			RETURN_IF_FAILED(RegisterPin(&m_Ultrasonic_Front_Left));

			tChar const * strDescSignalUltrasonic_Front_Center = pDescManager->GetMediaDescription("tSignalValue");
			RETURN_IF_POINTER_NULL(strDescSignalUltrasonic_Front_Center);
			cObjectPtr<IMediaType> pTypeSignalUltrasonic_Front_Center = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalUltrasonic_Front_Center, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
			RETURN_IF_FAILED(pTypeSignalUltrasonic_Front_Center->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescUltrasonic_Front_Center));
			RETURN_IF_FAILED(m_Ultrasonic_Front_Center.Create("ultrasonic_front_center", pTypeSignalUltrasonic_Front_Center, static_cast<IPinEventSink*> (this)));
			RETURN_IF_FAILED(RegisterPin(&m_Ultrasonic_Front_Center));

		    tChar const * strDescSignalUltrasonic_Rear_Center = pDescManager->GetMediaDescription("tSignalValue");
			RETURN_IF_POINTER_NULL(strDescSignalUltrasonic_Rear_Center);
			cObjectPtr<IMediaType> pTypeSignalUltrasonic_Rear_Center = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalUltrasonic_Rear_Center, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
			RETURN_IF_FAILED(pTypeSignalUltrasonic_Rear_Center->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescUltrasonic_Rear_Center));
			RETURN_IF_FAILED(m_Ultrasonic_Rear_Center.Create("ultrasonic_rear_center", pTypeSignalUltrasonic_Rear_Center, static_cast<IPinEventSink*> (this)));
			RETURN_IF_FAILED(RegisterPin(&m_Ultrasonic_Rear_Center));

			tChar const * strDescSignalUltrasonic_Rear_Left = pDescManager->GetMediaDescription("tSignalValue");
			RETURN_IF_POINTER_NULL(strDescSignalUltrasonic_Rear_Left);
			cObjectPtr<IMediaType> pTypeSignalUltrasonic_Rear_Left = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalUltrasonic_Rear_Left, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
			RETURN_IF_FAILED(pTypeSignalUltrasonic_Rear_Left->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescUltrasonic_Rear_Left));
			RETURN_IF_FAILED(m_Ultrasonic_Rear_Left.Create("ultrasonic_rear_Left", pTypeSignalUltrasonic_Rear_Left, static_cast<IPinEventSink*> (this)));
			RETURN_IF_FAILED(RegisterPin(&m_Ultrasonic_Rear_Left));

			tChar const * strDescSignalUltrasonic_Side_Left = pDescManager->GetMediaDescription("tSignalValue");
			RETURN_IF_POINTER_NULL(strDescSignalUltrasonic_Side_Left);
			cObjectPtr<IMediaType> pTypeSignalUltrasonic_Side_Left = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalUltrasonic_Side_Left, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
			RETURN_IF_FAILED(pTypeSignalUltrasonic_Side_Left->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescUltrasonic_Side_Left));
			RETURN_IF_FAILED(m_Ultrasonic_Side_Left.Create("ultrasonic_side_Left", pTypeSignalUltrasonic_Side_Left, static_cast<IPinEventSink*> (this)));
			RETURN_IF_FAILED(RegisterPin(&m_Ultrasonic_Side_Left));

		    tChar const * strDescSignalUltrasonic_Rear_Right = pDescManager->GetMediaDescription("tSignalValue");
			RETURN_IF_POINTER_NULL(strDescSignalUltrasonic_Rear_Right);
			cObjectPtr<IMediaType> pTypeSignalUltrasonic_Rear_Right = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalUltrasonic_Rear_Right, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
			RETURN_IF_FAILED(pTypeSignalUltrasonic_Rear_Right->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescUltrasonic_Rear_Right));
			RETURN_IF_FAILED(m_Ultrasonic_Rear_Right.Create("ultrasonic_rear_Right", pTypeSignalUltrasonic_Rear_Right, static_cast<IPinEventSink*> (this)));
			RETURN_IF_FAILED(RegisterPin(&m_Ultrasonic_Rear_Right));

			tChar const * strDescSignaldistanceoverall = pDescManager->GetMediaDescription("tSignalValue");
			RETURN_IF_POINTER_NULL(strDescSignaldistanceoverall);
			cObjectPtr<IMediaType> pTypeSignaldistanceoverall = new cMediaType(0, 0, 0, "tSignalValue", strDescSignaldistanceoverall, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
			RETURN_IF_FAILED(pTypeSignaldistanceoverall->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescdistanceoverall));
			RETURN_IF_FAILED(m_distanceoverall.Create("Distance_Overall", pTypeSignaldistanceoverall, static_cast<IPinEventSink*> (this)));
			RETURN_IF_FAILED(RegisterPin(&m_distanceoverall));
			
			tChar const * strDescSignalstart = pDescManager->GetMediaDescription("tSignalValue"); //tBoolSignalValue
			RETURN_IF_POINTER_NULL(strDescSignalstart);
			cObjectPtr<IMediaType> pTypeSignalstart = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalstart, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
			RETURN_IF_FAILED(pTypeSignalstart->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescstart));
			RETURN_IF_FAILED(m_start.Create("start_ID", pTypeSignalstart, static_cast<IPinEventSink*> (this)));
			RETURN_IF_FAILED(RegisterPin(&m_start));
			
			// output pins 
			tChar const * strDescSignalsteer = pDescManager->GetMediaDescription("tSignalValue"); //tBoolSignalValue
			RETURN_IF_POINTER_NULL(strDescSignalsteer);
			cObjectPtr<IMediaType> pTypeSignalsteer = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalsteer, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
			RETURN_IF_FAILED(pTypeSignalsteer->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescsteer));
			RETURN_IF_FAILED(m_osteer.Create("Steering", pTypeSignalsteer, static_cast<IPinEventSink*> (this)));
			RETURN_IF_FAILED(RegisterPin(&m_osteer));

			tChar const * strDescSignalaccelerate = pDescManager->GetMediaDescription("tSignalValue"); 
			RETURN_IF_POINTER_NULL(strDescSignalaccelerate);
			cObjectPtr<IMediaType> pTypeSignalaccelerate = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalaccelerate, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
			RETURN_IF_FAILED(pTypeSignalaccelerate->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescaccelerate));
			RETURN_IF_FAILED(m_oaccelerate.Create("accelerate", pTypeSignalaccelerate, static_cast<IPinEventSink*> (this)));
			RETURN_IF_FAILED(RegisterPin(&m_oaccelerate));

			tChar const * strDescSignalfinishflag = pDescManager->GetMediaDescription("tSignalValue"); 
			RETURN_IF_POINTER_NULL(strDescSignalfinishflag);
			cObjectPtr<IMediaType> pTypeSignalfinishflag = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalfinishflag, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
			RETURN_IF_FAILED(pTypeSignalfinishflag->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescfinishflag));
			RETURN_IF_FAILED(m_ofinishflag.Create("Pullout_Status", pTypeSignalfinishflag, static_cast<IPinEventSink*> (this)));
			RETURN_IF_FAILED(RegisterPin(&m_ofinishflag));


			/*tChar const * med_des_m_headlight_left = pDescManager->GetMediaDescription("tBoolSignalValue");
			RETURN_IF_POINTER_NULL(med_des_m_headlight_left);
			cObjectPtr<IMediaType> med_typ_m_headlight_left_loc	= new cMediaType(0, 0, 0, "tBoolSignalValue", med_des_m_headlight_left, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
			RETURN_IF_FAILED(med_typ_m_headlight_left_loc->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&med_typ_m_headlight_left));
			RETURN_IF_FAILED(m_headlight_left.Create("Indicator_Left", med_typ_m_headlight_left_loc, static_cast<IPinEventSink*> (this)));
			RETURN_IF_FAILED(RegisterPin(&m_headlight_left));*/

			tChar const * med_des_m_headlight = pDescManager->GetMediaDescription("tBoolSignalValue");
			RETURN_IF_POINTER_NULL(med_des_m_headlight);
			cObjectPtr<IMediaType> med_typ_m_headlight_loc	= new cMediaType(0, 0, 0, "tBoolSignalValue", med_des_m_headlight, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
			RETURN_IF_FAILED(med_typ_m_headlight_loc->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&med_typ_m_headlight));
			RETURN_IF_FAILED(m_headlight.Create("Reverse_Light", med_typ_m_headlight_loc, static_cast<IPinEventSink*> (this)));
			RETURN_IF_FAILED(RegisterPin(&m_headlight));

		}

		else if (eStage == StageNormal)
		{
			

		}
		else if (eStage == StageGraphReady)
		{

		}

		RETURN_NOERROR;

}

tResult cpullout::Start(__exception)
		{
			return cFilter::Start(__exception_ptr);
		}

tResult cpullout::Stop(__exception)
		{
			return cFilter::Stop(__exception_ptr);
		}

tResult cpullout::Shutdown(tInitStage eStage, __exception)
		{
			if (eStage == StageNormal)
			{

			}
			return cFilter::Shutdown(eStage, __exception_ptr);
		}



tResult cpullout::PropertyChanged(const char* strProperty)
		{
			ReadProperties(strProperty);
			RETURN_NOERROR;
		}
		
tResult cpullout::ReadProperties(const tChar* strPropertyName)
		{
			if (NULL == strPropertyName || cString::IsEqual(strPropertyName, DISTANCE_TRAJ1))
			{
				distance = static_cast<tFloat32> (GetPropertyFloat(DISTANCE_TRAJ1));
			}

			if (NULL == strPropertyName || cString::IsEqual(strPropertyName, DISTANCE_TRAJ2))
			{
				distance2 = static_cast<tFloat32> (GetPropertyFloat(DISTANCE_TRAJ2));
			}

			if (NULL == strPropertyName || cString::IsEqual(strPropertyName, DISTANCE_TRAJ3))
			{
				distance3 = static_cast<tFloat32> (GetPropertyFloat(DISTANCE_TRAJ3));
			}

			if (NULL == strPropertyName || cString::IsEqual(strPropertyName, DISTANCE_TRAJ4))
			{
				distance4 = static_cast<tFloat32> (GetPropertyFloat(DISTANCE_TRAJ4));
			}
			
			if (NULL == strPropertyName || cString::IsEqual(strPropertyName, DISTANCE_TRAJ5))
			{
				distance5 = static_cast<tFloat32> (GetPropertyFloat(DISTANCE_TRAJ5));
			}
			if (NULL == strPropertyName || cString::IsEqual(strPropertyName, DISTANCE_TRAJ6))
			{
				distance6 = static_cast<tFloat32> (GetPropertyFloat(DISTANCE_TRAJ6));
			}
			if (NULL == strPropertyName || cString::IsEqual(strPropertyName, DISTANCE_TRAJ7))
			{
				distance7 = static_cast<tFloat32> (GetPropertyFloat(DISTANCE_TRAJ7));
			}
			if (NULL == strPropertyName || cString::IsEqual(strPropertyName, DISTANCE_TRAJ8))
			{
				distance8 = static_cast<tFloat32> (GetPropertyFloat(DISTANCE_TRAJ8));
			}
			if (NULL == strPropertyName || cString::IsEqual(strPropertyName, DISTANCE_TRAJ9))
			{
				distance9 = static_cast<tFloat32> (GetPropertyFloat(DISTANCE_TRAJ9));
			}
			if (NULL == strPropertyName || cString::IsEqual(strPropertyName, DISTANCE_TRAJ10))
			{
				distance10 = static_cast<tFloat32> (GetPropertyFloat(DISTANCE_TRAJ10));
			}
			if (NULL == strPropertyName || cString::IsEqual(strPropertyName, DISTANCE_TRAJ11))
			{
				distance11 = static_cast<tFloat32> (GetPropertyFloat(DISTANCE_TRAJ11));
			}

			RETURN_NOERROR;
		}
		
tResult cpullout::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{

	RETURN_IF_POINTER_NULL(pMediaSample);
	RETURN_IF_POINTER_NULL(pSource);


	if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
	{
 
		tUInt32 timeStamp = 0;
		if (pSource == &m_start)
		{
			cObjectPtr<IMediaCoder> pCoderInput;
			RETURN_IF_FAILED(m_pDescstart->Lock(pMediaSample, &pCoderInput));
			pCoderInput->Get("f32Value", (tVoid*)&start);
			pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
			m_pDescstart->Unlock(pCoderInput);
                     //LOG_INFO(cString::Format("pullout start, startflag=%f", start));

			//else accelerate =90;
			//else if (start==pulloutleft_start) pulloutleft (timeStamp);
			//else if (start==pulloutright_start) pulloutright (timeStamp);

		}
		else if (pSource == &m_Ultrasonic_Front_Left)
		{

			cObjectPtr<IMediaCoder> pCoderInput;
			RETURN_IF_FAILED(m_pDescUltrasonic_Front_Left->Lock(pMediaSample, &pCoderInput));
			pCoderInput->Get("f32Value", (tVoid*)&Ultrasonic_Front_Left);
			pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
			m_pDescUltrasonic_Front_Left->Unlock(pCoderInput);
		}
		else if (pSource == &m_Ultrasonic_Front_Center)
		{

			cObjectPtr<IMediaCoder> pCoderInput;
			RETURN_IF_FAILED(m_pDescUltrasonic_Front_Center->Lock(pMediaSample, &pCoderInput));
			pCoderInput->Get("f32Value", (tVoid*)&Ultrasonic_Front_Center);
			pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
			m_pDescUltrasonic_Front_Center->Unlock(pCoderInput);
		}

		else if (pSource == &m_Ultrasonic_Side_Left)
		{

			cObjectPtr<IMediaCoder> pCoderInput;
			RETURN_IF_FAILED(m_pDescUltrasonic_Side_Left->Lock(pMediaSample, &pCoderInput));
			pCoderInput->Get("f32Value", (tVoid*)&Ultrasonic_Side_Left);
			pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
			m_pDescUltrasonic_Side_Left->Unlock(pCoderInput);
		}

		else if (pSource == &m_Ultrasonic_Rear_Center)
		{

			cObjectPtr<IMediaCoder> pCoderInput;
			RETURN_IF_FAILED(m_pDescUltrasonic_Rear_Center->Lock(pMediaSample, &pCoderInput));
			pCoderInput->Get("f32Value", (tVoid*)&Ultrasonic_Rear_Center);
			pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
			m_pDescUltrasonic_Rear_Center->Unlock(pCoderInput);
		}

		else if (pSource == &m_Ultrasonic_Rear_Right)
		{

			cObjectPtr<IMediaCoder> pCoderInput;
			RETURN_IF_FAILED(m_pDescUltrasonic_Rear_Right->Lock(pMediaSample, &pCoderInput));
			pCoderInput->Get("f32Value", (tVoid*)&Ultrasonic_Rear_Right);
			pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
			m_pDescUltrasonic_Rear_Right->Unlock(pCoderInput);
		}
        else if (pSource == &m_Ultrasonic_Rear_Left)
		{

			cObjectPtr<IMediaCoder> pCoderInput;
			RETURN_IF_FAILED(m_pDescUltrasonic_Rear_Left->Lock(pMediaSample, &pCoderInput));
			pCoderInput->Get("f32Value", (tVoid*)&Ultrasonic_Rear_Left);
			pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
			m_pDescUltrasonic_Rear_Left->Unlock(pCoderInput);
		}
		
		else if (pSource == &m_distanceoverall)
		{
			//LOG_INFO("Vinoth distance info");
			cObjectPtr<IMediaCoder> pCoderInput;
			RETURN_IF_FAILED(m_pDescdistanceoverall->Lock(pMediaSample, &pCoderInput));
			pCoderInput->Get("f32Value", (tVoid*)&distanceoverall);
			pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
			m_pDescdistanceoverall->Unlock(pCoderInput);
            if (((fabs(START_PULLOVER_PARALLEL - start) <= FLT_EPSILONA) || (fabs(START_PULLOVER_RIGHT - start) < FLT_EPSILONA)
        			||(fabs(START_PULLOVER_LEFT - start) < FLT_EPSILONA))
            		&& !first_sample)
			{
				        first_sample=true;
                		dist_at_start = distanceoverall;
                		if (Ultrasonic_Front_Center > 0.35) distance1 = 0.0f;  // no need to reverse as front space is free
                		else if (Ultrasonic_Rear_Center > 0.3) distance1 = distance; // enough space to reverse
                		else distance1 = Ultrasonic_Rear_Center-0.1; // not expected distance but can go back this much
			}  
			  if (abs(START_PULLOVER_PARALLEL - start) <= FLT_EPSILONA) pullout_parallel (timeStamp);
			  if (abs(START_PULLOVER_RIGHT - start) <= FLT_EPSILONA) pulloutright (timeStamp);
			  if (abs(START_PULLOVER_LEFT - start) <= FLT_EPSILONA) pulloutleft (timeStamp);
              //else if (start ==2.0 && Ultrasonic_Rear_Left > 1 && Ultrasonic_Rear_Right > 1 && Ultrasonic_Rear_Center > 1) pulloutleft (timeStamp);
              //else if (start ==3.0 && Ultrasonic_Rear_Left > 1 && Ultrasonic_Rear_Right > 1 && Ultrasonic_Rear_Center > 1) pulloutright(timeStamp);
              else { accelerate =90; steer=90;}
	    }
				
		else 
		{
			RETURN_ERROR(ERR_FAILED);
		}

	}
	RETURN_NOERROR;
}

tResult cpullout::pullout_parallel(tUInt32 timeStamp)
{
// 0-reverse , 0.6 , 0.6 ,0.2
if (first_sample==true)
{
	if (Ultrasonic_Side_Left > 0.45 && distanceoverall < (dist_at_start+distance1))
	{ 		
        active = pullout_parallel_active;
        var_headlight = tTrue;
        accelerate =99;
        steer=90;
        //timeflag = tTrue;
        //start_time = _clock->GetStreamTime();
        //end_time = _clock->GetStreamTime();
	}
	else if (Ultrasonic_Side_Left > 0.45 && Ultrasonic_Front_Center > 0.1 && Ultrasonic_Front_Left >0.1 && distanceoverall >= (dist_at_start+distance1) && distanceoverall < (dist_at_start+distance1+distance2))
	{
		active = pullout_parallel_active;
		var_headlight = tFalse;
        accelerate = 81;
        steer=60;
	}
	else if (Ultrasonic_Front_Center > 0.2 && distanceoverall >= (dist_at_start+distance1+distance2) && distanceoverall < (dist_at_start+distance1+distance2+distance3))
	{
        accelerate = 81;
        var_headlight = tFalse;
        steer=120;
	}
	else if (Ultrasonic_Front_Center > 0.2 && distanceoverall >= (dist_at_start+distance1+distance2+distance3) && distanceoverall < (dist_at_start+distance1+distance2+distance3+distance4))
	{
        accelerate =81;
        var_headlight = tFalse;
        steer=90;
	}
	else if(distanceoverall > (dist_at_start+distance1+distance2+distance3+distance4))
	{
		active = pullout_finished;
		accelerate = 90;
		first_sample= false;
		start = PULLOVER_STATUS_DEFAULT;
		var_headlight = tFalse;
		flag_switch_left_right = tFalse;
		//LOG_INFO(cString::Format("maneuver right is completed, finishflag=%d", finishflag));
	}
	else {accelerate = 90; steer = 90.0;}
generateop(timeStamp);
}	
	RETURN_NOERROR;
}

tResult cpullout::pulloutleft(tUInt32 timeStamp)
{
// 0.25 0.9 0.65 0.3
if (first_sample==true)
{
	if(Ultrasonic_Rear_Left > 0.4 && Ultrasonic_Rear_Right > 0.3 && Ultrasonic_Rear_Center > 0.85 && distanceoverall < (dist_at_start+distance5))
	{ 
        active = pulloutleft_active;
        var_headlight = tTrue;
        accelerate =97;
        steer=90;
        timeflag = tTrue;
        start_time = _clock->GetStreamTime();
        //end_time = _clock->GetStreamTime();
	}
	else if (Ultrasonic_Rear_Right > 0.3 && Ultrasonic_Rear_Center > 0.3 && distanceoverall >= (dist_at_start+distance5) && distanceoverall < (dist_at_start+distance5+distance6))
	{
        accelerate = 99;
        var_headlight = tTrue;
        steer=60;
	}
	else if (Ultrasonic_Front_Center > 0.2 && distanceoverall >= (dist_at_start+distance5+distance6) && distanceoverall < (dist_at_start+distance5+distance6+distance7))
	{
        accelerate =81;
        var_headlight = tFalse;
        steer=120;
	}
	else if (Ultrasonic_Front_Center>0.2 && distanceoverall >= (dist_at_start+distance5+distance6+distance7) && distanceoverall < (dist_at_start+distance5+distance6+distance7+distance8))
	{
        accelerate =83;
        var_headlight = tFalse;
        steer=90;
	}
	else if(distanceoverall > (dist_at_start+distance5+distance6+distance7+distance8))
	{
		active = pullout_finished;
		accelerate = 90;
		first_sample= false;
		start = PULLOVER_STATUS_DEFAULT;
		var_headlight = tFalse;
		flag_switch_left_right = tFalse;
		//LOG_INFO(cString::Format("maneuver right is completed, finishflag=%d", finishflag));
	}
	else {accelerate = 90;steer = 90;}
generateop(timeStamp);
}	
	RETURN_NOERROR;
}

tResult cpullout::pulloutright(tUInt32 timeStamp)
{
if (first_sample==true)
{ //0.75, 0.7, 0.3
	if (Ultrasonic_Rear_Left > 0.4 && Ultrasonic_Rear_Right > 0.4 && Ultrasonic_Rear_Center > 0.5 && distanceoverall < (dist_at_start+distance9))
	{ 
        active = pulloutright_active;
        var_headlight = tTrue;
        accelerate =99;
        steer=120;
        timeflag = tTrue;
        start_time = _clock->GetStreamTime();
        //end_time = _clock->GetStreamTime();
	}
	else if (Ultrasonic_Front_Center>0.2 && distanceoverall >= (dist_at_start+distance9) && distanceoverall < (dist_at_start+distance9+distance10))
	{
		var_headlight = tFalse;
        accelerate = 80;
        steer=60;
	}
	else if (Ultrasonic_Front_Center>0.2 && distanceoverall >= (dist_at_start+distance9+distance10) && distanceoverall < (dist_at_start+distance9+distance10+distance11))
	{
		var_headlight = tFalse;
        accelerate =81;
        steer=90;
	}
	else if (distanceoverall > (dist_at_start+distance9+distance10+distance11))
	{
		active = pullout_finished;
		accelerate = 90;
		first_sample= false;
		start = PULLOVER_STATUS_DEFAULT;
		var_headlight = tFalse;
		flag_switch_left_right = tFalse;
		//LOG_INFO(cString::Format("maneuver right is completed, finishflag=%d", finishflag));
	}
	else {accelerate = 90;steer = 90;}
generateop(timeStamp);
}	
	RETURN_NOERROR;
}


tResult cpullout::generateop(tUInt32 timeStamp)
{
	//create new media sample
	cObjectPtr<IMediaSample> pMediaSampleaccelerate;
	cObjectPtr<IMediaSample> pMediaSamplesteer;
	cObjectPtr<IMediaSample> pMediaSamplefinishflag;
	cObjectPtr<IMediaSample> pMediaSampleheadlight;
	//cObjectPtr<IMediaSample> pMediaSampleheadlight_left;

	AllocMediaSample((tVoid**)&pMediaSampleheadlight);
	//AllocMediaSample((tVoid**)&pMediaSampleheadlight_left);
	AllocMediaSample((tVoid**)&pMediaSampleaccelerate);
	AllocMediaSample((tVoid**)&pMediaSamplesteer);
	AllocMediaSample((tVoid**)&pMediaSamplefinishflag);

    cObjectPtr<IMediaSerializer> pSerializerfinishflag;
	m_pDescfinishflag->GetMediaSampleSerializer(&pSerializerfinishflag);
	tInt nSizefinishflag = pSerializerfinishflag->GetDeserializedSize();
	pMediaSamplefinishflag->AllocBuffer(nSizefinishflag);
	cObjectPtr<IMediaCoder> pCoderOutputfinishflag;
	m_pDescfinishflag->WriteLock(pMediaSamplefinishflag, &pCoderOutputfinishflag);
	pCoderOutputfinishflag->Set("f32Value", (tVoid*)&(active));//bValue
	pCoderOutputfinishflag->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
	m_pDescfinishflag->Unlock(pCoderOutputfinishflag);
	pMediaSamplefinishflag->SetTime(_clock->GetStreamTime());
	m_ofinishflag.Transmit(pMediaSamplefinishflag);

   /* cObjectPtr<IMediaSerializer> pSerializereheadlight_left;
    med_typ_m_headlight_left->GetMediaSampleSerializer(&pSerializereheadlight_left);
	tInt nSizeheadlight_left = pSerializereheadlight_left->GetDeserializedSize();
	pMediaSampleheadlight_left->AllocBuffer(nSizeheadlight_left);
	cObjectPtr<IMediaCoder> pCoderOutputheadlight_left;
	med_typ_m_headlight_left->WriteLock(pMediaSampleheadlight_left, &pCoderOutputheadlight_left);
	pCoderOutputheadlight_left->Set("bValue", (tVoid*)&(var_headlight_left));
	pCoderOutputheadlight_left->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
	med_typ_m_headlight_left->Unlock(pCoderOutputheadlight_left);
	pMediaSampleheadlight_left->SetTime(_clock->GetStreamTime());
	m_headlight_left.Transmit(pMediaSampleheadlight_left);*/

	cObjectPtr<IMediaSerializer> pSerializereheadlight;
    med_typ_m_headlight->GetMediaSampleSerializer(&pSerializereheadlight);
	tInt nSizeheadlight = pSerializereheadlight->GetDeserializedSize();
	pMediaSampleheadlight->AllocBuffer(nSizeheadlight);
	cObjectPtr<IMediaCoder> pCoderOutputheadlight;
	med_typ_m_headlight->WriteLock(pMediaSampleheadlight, &pCoderOutputheadlight);
	pCoderOutputheadlight->Set("bValue", (tVoid*)&(var_headlight));
	pCoderOutputheadlight->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
	med_typ_m_headlight->Unlock(pCoderOutputheadlight);
	pMediaSampleheadlight->SetTime(_clock->GetStreamTime());
	m_headlight.Transmit(pMediaSampleheadlight);

	cObjectPtr<IMediaSerializer> pSerializeraccelerate;
	m_pDescaccelerate->GetMediaSampleSerializer(&pSerializeraccelerate);
	tInt nSizeaccelerate = pSerializeraccelerate->GetDeserializedSize();
	pMediaSampleaccelerate->AllocBuffer(nSizeaccelerate);
	cObjectPtr<IMediaCoder> pCoderOutputaccelerate;
	m_pDescaccelerate->WriteLock(pMediaSampleaccelerate, &pCoderOutputaccelerate);
	pCoderOutputaccelerate->Set("f32Value", (tVoid*)&(accelerate));
	pCoderOutputaccelerate->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
	m_pDescaccelerate->Unlock(pCoderOutputaccelerate);
	pMediaSampleaccelerate->SetTime(_clock->GetStreamTime());
	m_oaccelerate.Transmit(pMediaSampleaccelerate);

    cObjectPtr<IMediaSerializer> pSerializersteer;
    m_pDescsteer->GetMediaSampleSerializer(&pSerializersteer);
    tInt nSizesteer = pSerializersteer->GetDeserializedSize();
    pMediaSamplesteer->AllocBuffer(nSizesteer);
	cObjectPtr<IMediaCoder> pCoderOutputsteer;
	m_pDescsteer->WriteLock(pMediaSamplesteer, &pCoderOutputsteer);
	pCoderOutputsteer->Set("f32Value", (tVoid*)&(steer));
	pCoderOutputsteer->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
	m_pDescsteer->Unlock(pCoderOutputsteer);
	pMediaSamplesteer->SetTime(_clock->GetStreamTime());
	m_osteer.Transmit(pMediaSamplesteer);


	RETURN_NOERROR;

}
