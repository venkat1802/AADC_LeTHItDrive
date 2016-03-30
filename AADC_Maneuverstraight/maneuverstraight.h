
#pragma once

#ifndef _MANEUVERSTRIGHT_H_
#define _MANEUVERSTRIGHT_H_

#include "stdafx.h"

#define OID_ADTF_MANEUVERSTRAIGHT "adtf.aadc.cmaneuverstraight"

class cmaneuverstraight : public adtf::cFilter
{
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_MANEUVERSTRAIGHT, "AADC Maneuver Straight", OBJCAT_DataFilter, "Maneuver Straight", 1, 0,0, "Beta Version");

    cInputPin m_distanceoverall;
	cInputPin m_start;
	cOutputPin m_ofinishflag;
	cObjectPtr<IMediaTypeDescription> m_pDescfinishflag;
	cObjectPtr<IMediaTypeDescription> m_pDescdistanceoverall;
	cObjectPtr<IMediaTypeDescription> m_pDescstart;

public:
	cmaneuverstraight(const tChar* __info);
    virtual ~cmaneuverstraight();
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
    tFloat32 start;
	tFloat32 dist_at_start;
	tBool first_sample,finishflag;

	tFloat32 distance_st1_straight,distance_st1_straight_afterT;

	tResult maneuver_straight(tUInt32 timeStamp);
	tResult generateop(tUInt32 timeStamp);

};

#endif

