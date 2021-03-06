﻿
#pragma once

#ifndef _MANEUVERRIGHT_H_
#define _MANEUVERRIGHT_H_

#include "stdafx.h"

#define OID_ADTF_pullout "adtf.aadc.pullout"
enum signal_id
{
	pullout_parallel_active=1, 
	pulloutleft_active,
	pulloutright_active,
	pullout_finished
};

enum start_id
{
	pullout_parallel_start=1, 
	pulloutleft_start,
	pulloutright_start,
    pullout_not_started
};

class cpullout : public adtf::cFilter
{

    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_pullout, "AADC Pull Out", OBJCAT_DataFilter, "Pull Out", 1, 0,0, "Beta Version");    

    cInputPin m_distanceoverall;
	//cInputPin m_yaw;
	cInputPin m_start;
	//cInputPin m_enable;
	
    cOutputPin m_osteer;	
    cOutputPin m_oaccelerate;
	cOutputPin m_ofinishflag;
	cOutputPin m_ostartstopflag;	
	cOutputPin m_headlight;
	//cOutputPin m_headlight_left;

	cInputPin m_Ultrasonic_Side_Left;                
    cInputPin m_Ultrasonic_Front_Center;
    cInputPin m_Ultrasonic_Front_Left;

	cInputPin m_Ultrasonic_Rear_Left;                
    cInputPin m_Ultrasonic_Rear_Center;
    cInputPin m_Ultrasonic_Rear_Right;

	cObjectPtr<IMediaTypeDescription> m_pDescUltrasonic_Side_Left;
    cObjectPtr<IMediaTypeDescription> m_pDescUltrasonic_Front_Center;
    cObjectPtr<IMediaTypeDescription> m_pDescUltrasonic_Front_Left;
	cObjectPtr<IMediaTypeDescription> m_pDescUltrasonic_Rear_Left;
    cObjectPtr<IMediaTypeDescription> m_pDescUltrasonic_Rear_Center;
    cObjectPtr<IMediaTypeDescription> m_pDescUltrasonic_Rear_Right;

	cObjectPtr<IMediaTypeDescription> med_typ_m_headlight;
	//cObjectPtr<IMediaTypeDescription> med_typ_m_headlight_left;

    cObjectPtr<IMediaTypeDescription> m_pDescdistanceoverall;
    cObjectPtr<IMediaTypeDescription> m_pDescstartstopflag;
	cObjectPtr<IMediaTypeDescription> m_pDescstart;
	
	
	cObjectPtr<IMediaTypeDescription> m_pDescsteer;
    cObjectPtr<IMediaTypeDescription> m_pDescaccelerate;
	cObjectPtr<IMediaTypeDescription> m_pDescfinishflag;


public:
    cpullout(const tChar* __info);
    virtual ~cpullout();
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

	tResult ReadProperties(const tChar* strPropertyName);

    tFloat32 distanceoverall;
	tBool time_flag;
	tBool start_time_flag,flag_switch_left_right;

	tBool var_headlight,timeflag,var_headlight_left;
	tTimeStamp start_time,end_time,time_diff,time_diff_last;
	//tFloat32 yaw_error;

	//tFloat32 whbase_length;
	tFloat32 dist_at_start;
	tBool first_sample;

	tFloat32 Ultrasonic_Side_Left;
    tFloat32 Ultrasonic_Front_Center;
    tFloat32 Ultrasonic_Front_Left;
	
	tFloat32 Ultrasonic_Rear_Left;
    tFloat32 Ultrasonic_Rear_Center;
    tFloat32 Ultrasonic_Rear_Right;

	tFloat32 accelerate;//, accel1, accel2, accel3;

	//tBool yaw_flag;
	tFloat32 steer;//, steer1,steer2, steer3,yaw_start;

	tFloat32    distance,distance1, distance2, distance3, distance4, distance5, distance6, distance7, distance8, distance9, distance10, distance11;

	//signal_id active;
	//start_id start;

	tFloat32 active;
	tFloat32 start;

	tResult pullout_parallel(tUInt32 timeStamp);
	tResult pulloutleft(tUInt32 timeStamp);
	tResult pulloutright(tUInt32 timeStamp);
	tResult generateop(tUInt32 timeStamp);

};

#endif

