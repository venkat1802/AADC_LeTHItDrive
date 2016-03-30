
#pragma once

#ifndef _MANEUVERRIGHT_H_
#define _MANEUVERRIGHT_H_

#include "stdafx.h"

#define OID_ADTF_MANEUVERRIGHT "adtf.aadc.cmaneuverright"

class cmaneuverright : public adtf::cFilter
{
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_MANEUVERRIGHT, "AADC Maneuver Right", OBJCAT_DataFilter, "Maneuver Right", 1, 0,0, "Beta Version");    

    cVideoPin m_oVideoInputPin;
    cInputPin m_distanceoverall;
	//cInputPin m_yaw;
	cInputPin m_start;
	//cInputPin m_enable;
	
    cOutputPin m_osteer;	
    cOutputPin m_oaccelerate;
	cOutputPin m_ofinishflag;
	cOutputPin m_ostartstopflag;	
	cOutputPin m_headlight;
	cOutputPin m_headlight_left;

	cObjectPtr<IMediaTypeDescription> med_typ_m_headlight;
	cObjectPtr<IMediaTypeDescription> med_typ_m_headlight_left;

    cObjectPtr<IMediaTypeDescription> m_pDescdistanceoverall;
    cObjectPtr<IMediaTypeDescription> m_pDescstartstopflag;
	cObjectPtr<IMediaTypeDescription> m_pDescstart;
	
	
	cObjectPtr<IMediaTypeDescription> m_pDescsteer;
    cObjectPtr<IMediaTypeDescription> m_pDescaccelerate;
	cObjectPtr<IMediaTypeDescription> m_pDescfinishflag;


public:
    cmaneuverright(const tChar* __info);
    virtual ~cmaneuverright();
    tResult PropertyChanged(const char* strProperty);

protected: // overwrites cFilter
    tResult Init(tInitStage eStage, __exception);
    tResult Start(__exception = NULL);
    tResult Stop(__exception = NULL);
    tResult Shutdown(tInitStage eStage, __exception);        
    tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);


private:    
   // tResult CreatePins( __exception = NULL);
    //tResult CreateOutputPins( __exception = NULL);

    tBitmapFormat      m_sInputFormat;
    tBool m_bFirstFrame;
    tResult ProcessInput(IMediaSample* pSample, tTimeStamp tsInputTime);
	tResult ReadProperties(const tChar* strPropertyName);

    tFloat32 distanceoverall;
	//tFloat32 yaw;
    tFloat32 start;
	//tBool enable;
	tBool time_flag;
	tBool finishflag;
	tBool start_time_flag;

	tBool var_headlight,timeflag,var_headlight_left;
	tTimeStamp start_time,end_time,time_diff,time_diff_last;
	//tFloat32 yaw_error;

	//tFloat32 whbase_length;
	tFloat32 dist_at_start;
	tBool first_sample;

	tFloat32 accelerate;//, accel1, accel2, accel3;

	tBool enable_corner_flag,corner_detected,imshowflag;
	tFloat32 steer;//, steer1,steer2, steer3,yaw_start;

	tFloat32    distance_st1,speed_st1,steer_st1;
	tFloat32    distance_curve,speed_curve,steer_curve;
	tFloat32    distance_st2,speed_st2,steer_st2;

	tFloat32    distance_st1_left,steer_left_angle,speed_st1_left;
	tFloat32    distance_curve_left,speed_left_curve,steer_left_curve;
	tFloat32    distance_st2_left,steer_st2_left,speed_st2_left;

	tFloat32    distance_st1_straight,steer_straight,speed_straight;

	tInt row1,row2,col1,col2;
	tInt  m_nThresholdValue;
	tResult maneuver_right(tUInt32 timeStamp);
	tResult maneuver_left(tUInt32 timeStamp);
	tResult maneuver_straight(tUInt32 timeStamp);
	tResult generateop(tUInt32 timeStamp);

};

#endif

