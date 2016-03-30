#include "stdafx.h"
//#include <cmath>
#include "maneuverright.h"

#define DISTANCE_TRAJ1 			"cmaneuverright::distance_st1"
#define STEER_TRAJ1 			"cmaneuverright::st1_steer"
#define SPEED_TRAJ1			 	"cmaneuverright::st1_speed"

#define DISTANCE_TRAJ2 			"cmaneuverright::distance_curve"
#define STEER_TRAJ2			 	"cmaneuverright::curve_steer"
#define SPEED_TRAJ2			 	"cmaneuverright::curve_speed"

#define DISTANCE_TRAJ3 			"cmaneuverright::distance_st2"
#define STEER_TRAJ3 			"cmaneuverright::st2_steer"
#define SPEED_TRAJ3			 	"cmaneuverright::st2_speed"

#define DISTANCE_TRAJ1_LEFT		 "cmaneuverleft::distance_st1"
#define STEER_LEFT               "cmaneuverleft::steer_st1"
#define SPEED_TRAJ1_LEFT		 "cmaneuverleft::speed_st1"

#define DISTANCE_TRAJ2_LEFT 	 "cmaneuverleft::distance_curve"
#define STEER_TRAJ2_LEFT		 "cmaneuverleft::steer_curve"
#define SPEED_TRAJ2_LEFT 	     "cmaneuverleft::speed_curve"

#define DISTANCE_TRAJ3_LEFT 	 "cmaneuverleft::distance_st2"
#define STEER_TRAJ3_LEFT		 "cmaneuverleft::steer_st2"
#define SPEED_TRAJ3_LEFT		 "cmaneuverleft::speed_st2"

#define DISTANCE_TRAJ1_STRAIGHT 			"cmaneuverstraight::distance_st1"
#define DISTANCE_TRAJ1_STRAIGHTSTEER 		"cmaneuverstraight::steer_st1"
#define DISTANCE_TRAJ1_STRAIGHTSPEED 		"cmaneuverstraight::speed_st1"

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
//#define FLT_EPSILON (tFloat32)0.01
#define MD_RANGE_ROWS_R1 "ManeuverDetection::ROW1"
#define MD_RANGE_ROWS_R2 "ManeuverDetection::ROW2"
#define MD_RANGE_COLS_C1 "ManeuverDetection::COL1"
#define MD_RANGE_COLS_C2 "ManeuverDetection::COL2"
#define MD_GREY_TRESHOLD "ManeuverDetection::GreyThresholdValue"
#define MD_ENABLE_IMSHOW  "ManeuverDetection::Enableimshow"

ADTF_FILTER_PLUGIN("AADC Maneuver Right", OID_ADTF_MANEUVERRIGHT, cmaneuverright)

cmaneuverright::cmaneuverright(const tChar* __info):cFilter(__info)
{
    SetPropertyBool(MD_ENABLE_IMSHOW,tFalse);
    SetPropertyBool(MD_ENABLE_IMSHOW NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(MD_ENABLE_IMSHOW NSSUBPROP_DESCRIPTION, "If true imshow will be enabled");

    SetPropertyInt(MD_GREY_TRESHOLD, 200);
    SetPropertyBool(MD_GREY_TRESHOLD NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(MD_GREY_TRESHOLD NSSUBPROP_DESCRIPTION, "The threshold value for canny to detect lines.");

    SetPropertyInt(MD_RANGE_ROWS_R1, 300);
    SetPropertyBool(MD_RANGE_ROWS_R1 NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(MD_RANGE_ROWS_R1 NSSUBPROP_DESCRIPTION, "MarkerDetection::The first row R1 to be taken (R1,) (R2,)");

    SetPropertyInt(MD_RANGE_ROWS_R2, 480);
    SetPropertyBool(MD_RANGE_ROWS_R2 NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(MD_RANGE_ROWS_R2 NSSUBPROP_DESCRIPTION,"MarkerDetection::The second row R2 to be taken (R1,) (R2,)");

    SetPropertyInt(MD_RANGE_COLS_C1, 300);
    SetPropertyBool(MD_RANGE_COLS_C1 NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(MD_RANGE_COLS_C1 NSSUBPROP_DESCRIPTION, "MarkerDetection::The first col C1 to be taken (,C1) (,C2)");

    SetPropertyInt(MD_RANGE_COLS_C2, 640);
    SetPropertyBool(MD_RANGE_COLS_C2 NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(MD_RANGE_COLS_C2 NSSUBPROP_DESCRIPTION, "MarkerDetection::The second col C2 to be taken (,C1) (,C2)");

	SetPropertyFloat(DISTANCE_TRAJ1,0.0);
    SetPropertyBool(DISTANCE_TRAJ1 NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(DISTANCE_TRAJ1 NSSUBPROP_DESCRIPTION, "the distance for the 1st trajectory");

	SetPropertyFloat(STEER_TRAJ1,90);
    SetPropertyBool(STEER_TRAJ1 NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(STEER_TRAJ1 NSSUBPROP_DESCRIPTION, "the steer for the 1st trajectory");

	SetPropertyFloat(SPEED_TRAJ1,83.5);
    SetPropertyBool(SPEED_TRAJ1 NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(SPEED_TRAJ1 NSSUBPROP_DESCRIPTION, "the speed for the 1st trajectory");

	SetPropertyFloat(DISTANCE_TRAJ2,1.3);
    SetPropertyBool(DISTANCE_TRAJ2 NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(DISTANCE_TRAJ2 NSSUBPROP_DESCRIPTION, "the distance for the curve trajectory");

	SetPropertyFloat(STEER_TRAJ2,120);
    SetPropertyBool(STEER_TRAJ2 NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(STEER_TRAJ2 NSSUBPROP_DESCRIPTION, "the steer for the curve trajectory");

    SetPropertyFloat(SPEED_TRAJ2,78);
    SetPropertyBool(SPEED_TRAJ2 NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(SPEED_TRAJ2 NSSUBPROP_DESCRIPTION, "the speed for the curve trajectory");

	SetPropertyFloat(DISTANCE_TRAJ3,0.3);
    SetPropertyBool(DISTANCE_TRAJ3 NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(DISTANCE_TRAJ3 NSSUBPROP_DESCRIPTION, "the distance for the straight after taking curve trajectory");

	SetPropertyFloat(STEER_TRAJ3,90);
    SetPropertyBool(STEER_TRAJ3 NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(STEER_TRAJ3 NSSUBPROP_DESCRIPTION, "the steer for the straight after taking curve trajectory");

	SetPropertyFloat(SPEED_TRAJ3,83);
    SetPropertyBool(SPEED_TRAJ3 NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(SPEED_TRAJ3 NSSUBPROP_DESCRIPTION, "the speed for the straight after taking curve trajectory");

	SetPropertyFloat(DISTANCE_TRAJ1_STRAIGHT,1);
    SetPropertyBool(DISTANCE_TRAJ1_STRAIGHT NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(DISTANCE_TRAJ1_STRAIGHT NSSUBPROP_DESCRIPTION, "the distance for the straight");

	SetPropertyFloat(DISTANCE_TRAJ1_STRAIGHTSTEER,90);
    SetPropertyBool(DISTANCE_TRAJ1_STRAIGHTSTEER NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(DISTANCE_TRAJ1_STRAIGHTSTEER NSSUBPROP_DESCRIPTION, "the steer for the straight");

	SetPropertyFloat(DISTANCE_TRAJ1_STRAIGHTSPEED,82);
    SetPropertyBool(DISTANCE_TRAJ1_STRAIGHTSPEED NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(DISTANCE_TRAJ1_STRAIGHTSPEED NSSUBPROP_DESCRIPTION, "the speed for the straight");

	SetPropertyFloat(DISTANCE_TRAJ1_LEFT,0.3);
    SetPropertyBool(DISTANCE_TRAJ1_LEFT NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(DISTANCE_TRAJ1_LEFT NSSUBPROP_DESCRIPTION, "the distance for the 1st trajectory left");

	SetPropertyFloat(STEER_LEFT,82);
    SetPropertyBool(STEER_LEFT NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(STEER_LEFT NSSUBPROP_DESCRIPTION, "the left steering for the 1st tragectory");

	SetPropertyFloat(SPEED_TRAJ1_LEFT,83);
    SetPropertyBool(SPEED_TRAJ1_LEFT NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(SPEED_TRAJ1_LEFT NSSUBPROP_DESCRIPTION, "the speed for the 1st trajectory left");

	SetPropertyFloat(DISTANCE_TRAJ2_LEFT,1.3);
    SetPropertyBool(DISTANCE_TRAJ2_LEFT NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(DISTANCE_TRAJ2_LEFT NSSUBPROP_DESCRIPTION, "the distance for the curve trajectory left");

    SetPropertyFloat(STEER_TRAJ2_LEFT,60);
    SetPropertyBool(STEER_TRAJ2_LEFT NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(STEER_TRAJ2_LEFT NSSUBPROP_DESCRIPTION, "the steer in curve for left turn");

    SetPropertyFloat(SPEED_TRAJ2_LEFT,78);
    SetPropertyBool(SPEED_TRAJ2_LEFT NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(SPEED_TRAJ2_LEFT NSSUBPROP_DESCRIPTION, "the speed in curve for left turn");

	SetPropertyFloat(DISTANCE_TRAJ3_LEFT,0.4);
    SetPropertyBool(DISTANCE_TRAJ3_LEFT NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(DISTANCE_TRAJ3_LEFT NSSUBPROP_DESCRIPTION, "the distance for the straight after taking curve trajectory left");

	SetPropertyFloat(STEER_TRAJ3_LEFT,90);
    SetPropertyBool(STEER_TRAJ3_LEFT NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(STEER_TRAJ3_LEFT NSSUBPROP_DESCRIPTION, "the steer for the straight after taking curve trajectory left");

	SetPropertyFloat(SPEED_TRAJ3_LEFT,83.5);
    SetPropertyBool(SPEED_TRAJ3_LEFT NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(SPEED_TRAJ3_LEFT NSSUBPROP_DESCRIPTION, "the speed for the straight after taking curve trajectory left");

	dist_at_start = 0;
    accelerate = 90.0;
	start= MANEUVER_INTERSECTION_DEFAULT;
	first_sample= tFalse;
	finishflag = tFalse;
	var_headlight = tFalse;
	var_headlight_left = tFalse;
	timeflag = tFalse;
	//start_time = end_time = _clock->GetStreamTime();
	time_diff = 3000000;
	time_diff_last = 1000000;
	//second_time_maenuverleft = tFalse;
	enable_corner_flag = tFalse;
	corner_detected = tFalse;
	ReadProperties(NULL);
}

cmaneuverright::~cmaneuverright()
{

}
tResult cmaneuverright::Init(tInitStage eStage, __exception)
{

	RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))


		if (eStage == StageFirst)
		{

			// create description manager
			cObjectPtr<IMediaDescriptionManager> pDescManager;
			RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**)&pDescManager, __exception_ptr));

	        RETURN_IF_FAILED(m_oVideoInputPin.Create("Video_Input_Maneuver", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
	        RETURN_IF_FAILED(RegisterPin(&m_oVideoInputPin));

			// get media type for input pins
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
			RETURN_IF_FAILED(m_start.Create("start", pTypeSignalstart, static_cast<IPinEventSink*> (this)));
			RETURN_IF_FAILED(RegisterPin(&m_start));
			
			// output pins 
			tChar const * strDescSignalsteer = pDescManager->GetMediaDescription("tSignalValue"); //tBoolSignalValue
			RETURN_IF_POINTER_NULL(strDescSignalsteer);
			cObjectPtr<IMediaType> pTypeSignalsteer = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalsteer, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
			RETURN_IF_FAILED(pTypeSignalsteer->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescsteer));
			RETURN_IF_FAILED(m_osteer.Create("Steering", pTypeSignalsteer, static_cast<IPinEventSink*> (this)));
			RETURN_IF_FAILED(RegisterPin(&m_osteer));

			tChar const * strDescSignalaccelerate = pDescManager->GetMediaDescription("tSignalValue"); //tBoolSignalValue
			RETURN_IF_POINTER_NULL(strDescSignalaccelerate);
			cObjectPtr<IMediaType> pTypeSignalaccelerate = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalaccelerate, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
			RETURN_IF_FAILED(pTypeSignalaccelerate->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescaccelerate));
			RETURN_IF_FAILED(m_oaccelerate.Create("accelerate", pTypeSignalaccelerate, static_cast<IPinEventSink*> (this)));
			RETURN_IF_FAILED(RegisterPin(&m_oaccelerate));

			tChar const * strDescSignalfinishflag = pDescManager->GetMediaDescription("tBoolSignalValue"); //tBoolSignalValue
			RETURN_IF_POINTER_NULL(strDescSignalfinishflag);
			cObjectPtr<IMediaType> pTypeSignalfinishflag = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalfinishflag, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
			RETURN_IF_FAILED(pTypeSignalfinishflag->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescfinishflag));
			RETURN_IF_FAILED(m_ofinishflag.Create("FinishFlag", pTypeSignalfinishflag, static_cast<IPinEventSink*> (this)));
			RETURN_IF_FAILED(RegisterPin(&m_ofinishflag));

			tChar const * strDescSignalstartstopflag = pDescManager->GetMediaDescription("tBoolSignalValue"); //tBoolSignalValue
			RETURN_IF_POINTER_NULL(strDescSignalstartstopflag);
			cObjectPtr<IMediaType> pTypeSignalstartstopflag = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalstartstopflag, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
			RETURN_IF_FAILED(pTypeSignalstartstopflag->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescstartstopflag));
			RETURN_IF_FAILED(m_ostartstopflag.Create("Flag_to_SpeedCon", pTypeSignalstartstopflag, static_cast<IPinEventSink*> (this)));
			RETURN_IF_FAILED(RegisterPin(&m_ostartstopflag));

			tChar const * med_des_m_headlight_left = pDescManager->GetMediaDescription("tBoolSignalValue");
			RETURN_IF_POINTER_NULL(med_des_m_headlight_left);
			cObjectPtr<IMediaType> med_typ_m_headlight_left_loc	= new cMediaType(0, 0, 0, "tBoolSignalValue", med_des_m_headlight_left, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
			RETURN_IF_FAILED(med_typ_m_headlight_left_loc->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&med_typ_m_headlight_left));
			RETURN_IF_FAILED(m_headlight_left.Create("Indicator_Left", med_typ_m_headlight_left_loc, static_cast<IPinEventSink*> (this)));
			RETURN_IF_FAILED(RegisterPin(&m_headlight_left));

			tChar const * med_des_m_headlight = pDescManager->GetMediaDescription("tBoolSignalValue");
			RETURN_IF_POINTER_NULL(med_des_m_headlight);
			cObjectPtr<IMediaType> med_typ_m_headlight_loc	= new cMediaType(0, 0, 0, "tBoolSignalValue", med_des_m_headlight, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
			RETURN_IF_FAILED(med_typ_m_headlight_loc->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&med_typ_m_headlight));
			RETURN_IF_FAILED(m_headlight.Create("Indicator_Right", med_typ_m_headlight_loc, static_cast<IPinEventSink*> (this)));
			RETURN_IF_FAILED(RegisterPin(&m_headlight));

		}

		else if (eStage == StageNormal)
		{
			m_bFirstFrame = tTrue;
			imshowflag = GetPropertyBool(MD_ENABLE_IMSHOW);

		}
		else if (eStage == StageGraphReady)
		{

		}

		RETURN_NOERROR;

}

tResult cmaneuverright::Start(__exception)
		{
			return cFilter::Start(__exception_ptr);
		}

tResult cmaneuverright::Stop(__exception)
		{
			return cFilter::Stop(__exception_ptr);
		}

tResult cmaneuverright::Shutdown(tInitStage eStage, __exception)
		{
			if (eStage == StageNormal)
			{

			}
			return cFilter::Shutdown(eStage, __exception_ptr);
		}



tResult cmaneuverright::PropertyChanged(const char* strProperty)
		{
			ReadProperties(strProperty);
			RETURN_NOERROR;
		}
		
tResult cmaneuverright::ReadProperties(const tChar* strPropertyName)
		{
			if (NULL == strPropertyName || cString::IsEqual(strPropertyName, MD_GREY_TRESHOLD))
			{
				m_nThresholdValue = GetPropertyInt(MD_GREY_TRESHOLD);
			}
			if (NULL == strPropertyName || cString::IsEqual(strPropertyName, MD_RANGE_ROWS_R1))
			{
				row1 = GetPropertyInt(MD_RANGE_ROWS_R1);
			}
			if (NULL == strPropertyName || cString::IsEqual(strPropertyName, MD_RANGE_ROWS_R2))
			{
				row2 = GetPropertyInt(MD_RANGE_ROWS_R2);
			}
			if (NULL == strPropertyName || cString::IsEqual(strPropertyName, MD_RANGE_COLS_C1))
			{
				col1 = GetPropertyInt(MD_RANGE_COLS_C1);
			}
			if (NULL == strPropertyName || cString::IsEqual(strPropertyName, MD_RANGE_COLS_C2))
			{
				col2 = GetPropertyInt(MD_RANGE_COLS_C2);
			}
			if (NULL == strPropertyName || cString::IsEqual(strPropertyName, DISTANCE_TRAJ1))
			{
				distance_st1 = static_cast<tFloat32> (GetPropertyFloat(DISTANCE_TRAJ1));
			}
			if (NULL == strPropertyName || cString::IsEqual(strPropertyName, STEER_TRAJ1))
			{
				steer_st1 = static_cast<tFloat32> (GetPropertyFloat(STEER_TRAJ1));
			}
			if (NULL == strPropertyName || cString::IsEqual(strPropertyName, SPEED_TRAJ1))
			{
				speed_st1 = static_cast<tFloat32> (GetPropertyFloat(SPEED_TRAJ1));
			}
			if (NULL == strPropertyName || cString::IsEqual(strPropertyName, DISTANCE_TRAJ2))
			{
				distance_curve = static_cast<tFloat32> (GetPropertyFloat(DISTANCE_TRAJ2));
			}
			if (NULL == strPropertyName || cString::IsEqual(strPropertyName, STEER_TRAJ2))
			{
				steer_curve = static_cast<tFloat32> (GetPropertyFloat(STEER_TRAJ2));
			}
			if (NULL == strPropertyName || cString::IsEqual(strPropertyName, SPEED_TRAJ2))
			{
				speed_curve = static_cast<tFloat32> (GetPropertyFloat(SPEED_TRAJ2));
			}
			if (NULL == strPropertyName || cString::IsEqual(strPropertyName, DISTANCE_TRAJ3))
			{
				distance_st2 = static_cast<tFloat32> (GetPropertyFloat(DISTANCE_TRAJ3));
			}
			if (NULL == strPropertyName || cString::IsEqual(strPropertyName, STEER_TRAJ3))
			{
				steer_st2 = static_cast<tFloat32> (GetPropertyFloat(STEER_TRAJ3));
			}
			if (NULL == strPropertyName || cString::IsEqual(strPropertyName, SPEED_TRAJ3))
			{
				speed_st2 = static_cast<tFloat32> (GetPropertyFloat(SPEED_TRAJ3));
			}
			if (NULL == strPropertyName || cString::IsEqual(strPropertyName, DISTANCE_TRAJ1_STRAIGHT))
			{
				distance_st1_straight = static_cast<tFloat32> (GetPropertyFloat(DISTANCE_TRAJ1_STRAIGHT));
			}
			if (NULL == strPropertyName || cString::IsEqual(strPropertyName, DISTANCE_TRAJ1_STRAIGHTSTEER))
			{
				steer_straight = static_cast<tFloat32> (GetPropertyFloat(DISTANCE_TRAJ1_STRAIGHTSTEER));
			}
			if (NULL == strPropertyName || cString::IsEqual(strPropertyName, DISTANCE_TRAJ1_STRAIGHTSPEED))
			{
				speed_straight = static_cast<tFloat32> (GetPropertyFloat(DISTANCE_TRAJ1_STRAIGHTSPEED));
			}
			if (NULL == strPropertyName || cString::IsEqual(strPropertyName, DISTANCE_TRAJ1_LEFT))
			{
				distance_st1_left = static_cast<tFloat32> (GetPropertyFloat(DISTANCE_TRAJ1_LEFT));
			}
			if (NULL == strPropertyName || cString::IsEqual(strPropertyName, STEER_LEFT))
			{
				steer_left_angle = static_cast<tFloat32> (GetPropertyFloat(STEER_LEFT));
			}
			if (NULL == strPropertyName || cString::IsEqual(strPropertyName, SPEED_TRAJ1_LEFT))
			{
				speed_st1_left = static_cast<tFloat32> (GetPropertyFloat(SPEED_TRAJ1_LEFT));
			}
			if (NULL == strPropertyName || cString::IsEqual(strPropertyName, DISTANCE_TRAJ2_LEFT))
			{
				distance_curve_left = static_cast<tFloat32> (GetPropertyFloat(DISTANCE_TRAJ2_LEFT));
			}
			if (NULL == strPropertyName || cString::IsEqual(strPropertyName, SPEED_TRAJ2_LEFT))
			{
				speed_left_curve = static_cast<tFloat32> (GetPropertyFloat(SPEED_TRAJ2_LEFT));
			}
			if (NULL == strPropertyName || cString::IsEqual(strPropertyName, STEER_TRAJ2_LEFT))
			{
				steer_left_curve = static_cast<tFloat32> (GetPropertyFloat(STEER_TRAJ2_LEFT));
			}
			if (NULL == strPropertyName || cString::IsEqual(strPropertyName, DISTANCE_TRAJ3_LEFT))
			{
				distance_st2_left = static_cast<tFloat32> (GetPropertyFloat(DISTANCE_TRAJ3_LEFT));
			}
			if (NULL == strPropertyName || cString::IsEqual(strPropertyName, SPEED_TRAJ3_LEFT))
			{
				speed_st2_left = static_cast<tFloat32> (GetPropertyFloat(SPEED_TRAJ3_LEFT));
			}
			if (NULL == strPropertyName || cString::IsEqual(strPropertyName, STEER_TRAJ3_LEFT))
			{
				steer_st2_left = static_cast<tFloat32> (GetPropertyFloat(STEER_TRAJ3_LEFT));
			}
			RETURN_NOERROR;
		}
		
tResult cmaneuverright::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
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
			if((fabs(MANEUVER_INTERSECTION_RIGHT - start) < FLT_EPSILON || fabs(MANEUVER_INTERSECTION_LEFT - start) < FLT_EPSILON ||
								fabs(MANEUVER_INTERSECTION_STRAIGHT - start) < FLT_EPSILON) && !first_sample)
			{
				first_sample=tTrue;
                dist_at_start = distanceoverall;
	            start_time = _clock->GetStreamTime();
	            timeflag = tTrue;
                LOG_INFO(cString::Format("Maneuver intresection is started, start=%f", start));
			}
		}
		
		else if (pSource == &m_distanceoverall)
		{
			//LOG_INFO("Vinoth distance info");
			cObjectPtr<IMediaCoder> pCoderInput;
			RETURN_IF_FAILED(m_pDescdistanceoverall->Lock(pMediaSample, &pCoderInput));
			pCoderInput->Get("f32Value", (tVoid*)&distanceoverall);
			pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
			m_pDescdistanceoverall->Unlock(pCoderInput);	
            if((fabs(MANEUVER_INTERSECTION_RIGHT - start) < FLT_EPSILON))
	             maneuver_right(timeStamp);
            else if((fabs(MANEUVER_INTERSECTION_LEFT - start) < FLT_EPSILON))
            	 maneuver_left(timeStamp);
            else if((fabs(MANEUVER_INTERSECTION_STRAIGHT - start) < FLT_EPSILON))
            	maneuver_straight(timeStamp);
            else
            	{}//LOG_INFO(cString::Format("Maneuver Right intresection maneuver is started, start=%f", start));
	    }
		else if (enable_corner_flag && pSource == &m_oVideoInputPin)  // process the video input here
		{
	        tTimeStamp InputTimeStamp;
	        InputTimeStamp = pMediaSample->GetTime();
            if (m_bFirstFrame)
            {
                cObjectPtr<IMediaType> pType;
                RETURN_IF_FAILED(m_oVideoInputPin.GetMediaType(&pType));
                cObjectPtr<IMediaTypeVideo> pTypeVideo;
                RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));
                const tBitmapFormat* pFormat = pTypeVideo->GetFormat();
                if (pFormat == NULL)
                {
                    LOG_ERROR("No Bitmap information found on pin \"input\"");
                    RETURN_ERROR(ERR_NOT_SUPPORTED);
                }
                m_sInputFormat.nPixelFormat = pFormat->nPixelFormat;
                m_sInputFormat.nWidth = pFormat->nWidth;
                m_sInputFormat.nHeight =  pFormat->nHeight;
                m_sInputFormat.nBitsPerPixel = pFormat->nBitsPerPixel;
                m_sInputFormat.nBytesPerLine = pFormat->nBytesPerLine;
                m_sInputFormat.nSize = pFormat->nSize;
                m_sInputFormat.nPaletteSize = pFormat->nPaletteSize;
                m_bFirstFrame = tFalse;
          }
            ProcessInput(pMediaSample, InputTimeStamp);
		}
		else 
		{
			RETURN_ERROR(ERR_FAILED);
		}

	}
	RETURN_NOERROR;
}

tResult cmaneuverright::ProcessInput(IMediaSample* pSample, tTimeStamp tsInputTime)
{
    RETURN_IF_POINTER_NULL(pSample);
    const tVoid* l_pSrcBuffer;
    IplImage* oImg = cvCreateImageHeader(cvSize(m_sInputFormat.nWidth, m_sInputFormat.nHeight), IPL_DEPTH_8U, 3);
    RETURN_IF_FAILED(pSample->Lock(&l_pSrcBuffer));
    oImg->imageData = (char*)l_pSrcBuffer;
    Mat image(cvarrToMat(oImg));
    cvReleaseImage(&oImg);  // c type interface . We need to call this function to release memory
    pSample->Unlock(l_pSrcBuffer);

    Mat image_right= image(cv::Range(row1, row2), cv::Range(col1, col2)).clone();
	Mat grey_image,greythresh,canny_image;
	vector<Vec2f> hough_lines;
	//GaussianBlur(image_right, image_right, Size(11,11), 0, 0, BORDER_DEFAULT); // Filter
	cvtColor(image_right ,grey_image,CV_BGR2GRAY);// Grey Image
	threshold(grey_image, greythresh, m_nThresholdValue, 255,THRESH_BINARY);// Generate Binary Image
	Canny(greythresh, canny_image, 0, 2, 3, false);// Detect Edges
	HoughLines(canny_image,hough_lines,1,CV_PI/180,10,0,0);
	if(imshowflag)
	{
		imshow("cannyimage",canny_image);
		imshow( "right image",image_right );
		waitKey(1);
	}
	if(0 == hough_lines.size())
	{
		corner_detected = tTrue;
		enable_corner_flag = tFalse;
		LOG_INFO("No lines detected in the right cornerpart");
	}
	RETURN_NOERROR;
}
tResult cmaneuverright::maneuver_right(tUInt32 timeStamp)
{
if (first_sample==tTrue)
{
	if (timeflag)
	{
		finishflag = tFalse;
		var_headlight = tTrue;
		accelerate = 90.0;
		steer = 90.0;
		end_time = _clock->GetStreamTime();
		if(time_diff < (end_time - start_time)) //3<0 ....3<3.2
			timeflag = tFalse;
		else
			accelerate = 90.0;
		dist_at_start = distanceoverall;
		corner_detected = tFalse;
	}
	else if (!corner_detected)
	{
		enable_corner_flag = tTrue;
        accelerate = speed_st1;
        steer =	steer_st1;
	}
	else if (distanceoverall >= (dist_at_start+distance_st1) && distanceoverall < (dist_at_start+distance_st1+distance_curve))
	{
        accelerate = speed_curve;
        steer=steer_curve;
	}
	else if (distanceoverall >= (dist_at_start+distance_st1+distance_curve) && distanceoverall < 
                (dist_at_start+distance_st1+distance_curve+distance_st2))
	{
        accelerate =speed_st2;
        steer=steer_st2;
	}
	else
	{
		finishflag = tTrue;
		accelerate = 90;
		steer = 90;
		first_sample= tFalse;
		start = MANEUVER_INTERSECTION_DEFAULT;
		var_headlight = tFalse;
		LOG_INFO(cString::Format("maneuver right is completed, finishflag=%d", finishflag));
	}
generateop(timeStamp);
}	
	RETURN_NOERROR;
}


tResult cmaneuverright::maneuver_straight(tUInt32 timeStamp)
{
if (first_sample==tTrue)
{
	if (timeflag)
	{
		finishflag = tFalse;
        var_headlight = tFalse;
        var_headlight_left = tFalse;
		accelerate = 90.0;
		steer = 90.0;
		end_time = _clock->GetStreamTime();
		if(time_diff < (end_time - start_time)) //3<0 ....3<3.2
			timeflag = tFalse;
		else
			accelerate = 90.0;
		dist_at_start = distanceoverall;
	}
	else if (distanceoverall < (dist_at_start+distance_st1_straight))
	{
        accelerate = speed_straight;
        steer = steer_straight;
	}
	else
	{
        finishflag = tTrue;
        accelerate = 90.0;
        steer = 90.0;
        first_sample= tFalse;
        start = MANEUVER_INTERSECTION_DEFAULT;
        var_headlight = tFalse;
        var_headlight_left = tFalse;
        LOG_INFO(cString::Format("maneuver straight is completed, finishflag=%d", finishflag));
	}
generateop(timeStamp);
}
	RETURN_NOERROR;
}

tResult cmaneuverright::maneuver_left(tUInt32 timeStamp)
{
if (first_sample==tTrue)
{
	if (timeflag)
	{
		finishflag = tFalse;
		var_headlight_left = tTrue;
		accelerate = 90.0;
		steer = steer_left_angle;
		end_time = _clock->GetStreamTime();
		if(time_diff < (end_time - start_time)) //3<0 ....3<3.2
			timeflag = tFalse;
		else
			accelerate = 90.0;
		dist_at_start = distanceoverall;
	}
	else if (distanceoverall < (dist_at_start+distance_st1_left))
	{
        accelerate = speed_st1_left;
        steer= steer_left_angle;
	}
	else if (distanceoverall >= (dist_at_start+distance_st1_left)
			&& distanceoverall < (dist_at_start+distance_st1_left+distance_curve_left))
	{
        accelerate = speed_left_curve;
        steer=steer_left_curve;
	}
	else if (distanceoverall >= (dist_at_start+distance_st1_left+distance_curve_left) && distanceoverall <
                (dist_at_start+distance_st1_left+distance_curve_left+distance_st2_left))
	{
        accelerate = speed_st2_left;
        steer = steer_st2_left;
	}
	else
	{
		//second_time_maenuverleft = !second_time_maenuverleft;
		finishflag = tTrue;
		accelerate = 90;
		steer = 90;
		first_sample= tFalse;
		start = MANEUVER_INTERSECTION_DEFAULT;
		var_headlight_left = tFalse;
		LOG_INFO(cString::Format("maneuver left is completed, finishflag=%d", finishflag));
	}
generateop(timeStamp);
}
	RETURN_NOERROR;
}

tResult cmaneuverright::generateop(tUInt32 timeStamp)
{
	//create new media sample
	cObjectPtr<IMediaSample> pMediaSampleaccelerate;
	cObjectPtr<IMediaSample> pMediaSamplesteer;
	cObjectPtr<IMediaSample> pMediaSamplefinishflag;
	cObjectPtr<IMediaSample> pMediaSamplestartstopflag;
	cObjectPtr<IMediaSample> pMediaSampleheadlight;
	cObjectPtr<IMediaSample> pMediaSampleheadlight_left;

	AllocMediaSample((tVoid**)&pMediaSampleheadlight);
	AllocMediaSample((tVoid**)&pMediaSampleheadlight_left);
	AllocMediaSample((tVoid**)&pMediaSampleaccelerate);
	AllocMediaSample((tVoid**)&pMediaSamplesteer);
	AllocMediaSample((tVoid**)&pMediaSamplefinishflag);
	AllocMediaSample((tVoid**)&pMediaSamplestartstopflag);

    cObjectPtr<IMediaSerializer> pSerializereheadlight_left;
    med_typ_m_headlight_left->GetMediaSampleSerializer(&pSerializereheadlight_left);
	tInt nSizeheadlight_left = pSerializereheadlight_left->GetDeserializedSize();
	pMediaSampleheadlight_left->AllocBuffer(nSizeheadlight_left);
	cObjectPtr<IMediaCoder> pCoderOutputheadlight_left;
	med_typ_m_headlight_left->WriteLock(pMediaSampleheadlight_left, &pCoderOutputheadlight_left);
	pCoderOutputheadlight_left->Set("bValue", (tVoid*)&(var_headlight_left));
	pCoderOutputheadlight_left->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
	med_typ_m_headlight_left->Unlock(pCoderOutputheadlight_left);
	pMediaSampleheadlight_left->SetTime(_clock->GetStreamTime());
	m_headlight_left.Transmit(pMediaSampleheadlight_left);

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

    cObjectPtr<IMediaSerializer> pSerializerfinishflag;
	m_pDescfinishflag->GetMediaSampleSerializer(&pSerializerfinishflag);
	tInt nSizefinishflag = pSerializerfinishflag->GetDeserializedSize();
	pMediaSamplefinishflag->AllocBuffer(nSizefinishflag);
	cObjectPtr<IMediaCoder> pCoderOutputfinishflag;
	m_pDescfinishflag->WriteLock(pMediaSamplefinishflag, &pCoderOutputfinishflag);
	pCoderOutputfinishflag->Set("bValue", (tVoid*)&(finishflag));//bValue
	pCoderOutputfinishflag->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
	m_pDescfinishflag->Unlock(pCoderOutputfinishflag);
	pMediaSamplefinishflag->SetTime(_clock->GetStreamTime());
	m_ofinishflag.Transmit(pMediaSamplefinishflag);
	//RETURN_NOERROR;

	cObjectPtr<IMediaSerializer> pSerializerstartstopflag;
	m_pDescstartstopflag->GetMediaSampleSerializer(&pSerializerstartstopflag);
	tInt nSizestartstopflag = pSerializerstartstopflag->GetDeserializedSize();
	pMediaSamplestartstopflag->AllocBuffer(nSizestartstopflag);
	cObjectPtr<IMediaCoder> pCoderOutputstartstopflag;
	m_pDescstartstopflag->WriteLock(pMediaSamplestartstopflag, &pCoderOutputstartstopflag);
	tBool final_variable = !finishflag;
	pCoderOutputstartstopflag->Set("bValue", (tVoid*)&(final_variable));//bValue
	pCoderOutputstartstopflag->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
	m_pDescstartstopflag->Unlock(pCoderOutputstartstopflag);
	pMediaSamplestartstopflag->SetTime(_clock->GetStreamTime());
	m_ostartstopflag.Transmit(pMediaSamplestartstopflag);
	RETURN_NOERROR;

}
