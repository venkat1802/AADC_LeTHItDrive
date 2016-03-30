
#pragma once

#ifndef _ADAPTIVECRUISECONTROL_H_
#define _ADAPTIVECRUISECONTROL_H_

#include "stdafx.h"

#define OID_ADTF_ADAPTIVECRUISECONTROL "adtf.aadc.cadaptivecruisecontrol"

class cadaptivecruisecontrol : public adtf::cFilter
{
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_ADAPTIVECRUISECONTROL, "AADC Adaptive Cruise Control", OBJCAT_DataFilter, "Adaptive Cruise Control", 1, 0,0, "Beta Version");    

    cInputPin m_ultrasonicfrontmid;
	cInputPin m_velocity;


    cOutputPin m_accaccel;	
    cOutputPin m_accactiveflag;
		
    cObjectPtr<IMediaTypeDescription> m_pDescultrasonicfrontmid;
    cObjectPtr<IMediaTypeDescription> m_pDescvelocity;
	cObjectPtr<IMediaTypeDescription> m_pDescaccaccel;
    cObjectPtr<IMediaTypeDescription> m_pDescaccactiveflag;
	

public:
    cadaptivecruisecontrol(const tChar* __info);
    virtual ~cadaptivecruisecontrol();
	map <tFloat32, tFloat32> lookup;
	tFloat32 ultrasonic, velocity, accaccel;
	tBool accactiveflag;
	
	tFloat32 last_distance, expected_dist, last_vel, des_speed, req_speed, set_distance, current_distance, timestep;
   // tResult PropertyChanged(const char* strProperty);

protected: // overwrites cFilter
    tResult Init(tInitStage eStage, __exception);
    tResult Start(__exception = NULL);
    tResult Stop(__exception = NULL);
    tResult Shutdown(tInitStage eStage, __exception);        
    tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);


private:    
   // tResult CreatePins( __exception = NULL);
    //tResult CreateOutputPins( __exception = NULL);

	//tResult ReadProperties(const tChar* strPropertyName);	
	
	tResult acc_calculation(tUInt32 timeStamp);
	tResult generateop(tUInt32 timeStamp);
        tResult ReadProperties(const tChar* strPropertyName);
        tResult PropertyChanged(const char* strProperty);

};

#endif

