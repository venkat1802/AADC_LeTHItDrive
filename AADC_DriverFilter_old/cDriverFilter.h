#pragma once

#ifndef _DRIVERFILTER_H_
#define _DRIVERFILTER_H_

#include "stdafx.h"

#define OID_ADTF_DRIVERFILTER "adtf.aadc.cdriverfilter"

class cdriverfilter : public adtf::cFilter
{
    //ADTF_DECLARE_FILTER_VERSION(OID_ADTF_SPEEDCONTROLLER, "AADC SPEED CONTROLLER ", OBJCAT_DataFilter);    
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_DRIVERFILTER, "AADC DRIVER FILTER", OBJCAT_DataFilter, "Driver Filter Controller", 1, 0,0, "Beta Version");

    cInputPin Emergency_Brake_Flag;      
    cInputPin cur_man_complete_Flag;   
    cInputPin     drive_struct;
     cOutputPin m_readystate;
    cOutputPin m_completestate;
     cOutputPin m_maneuverid;
	cOutputPin m_accelerate;
cInputPin m_Ultrasonic_Front_Center;
	
    cObjectPtr<IMediaTypeDescription> med_typ_Emergency_Brake_Flag;
	cObjectPtr<IMediaTypeDescription> med_typ_m_accelerate;
	cObjectPtr<IMediaTypeDescription> mediatype_drive_struct;
	cObjectPtr<IMediaTypeDescription> mediatype_maneuverid;
	cObjectPtr<IMediaTypeDescription> mediatype_maneuversetstate;
	cObjectPtr<IMediaTypeDescription> med_typ_cur_man_complete_Flag;
     cObjectPtr<IMediaTypeDescription> med_typ_readystate;
     cObjectPtr<IMediaTypeDescription> med_typ_completestate;
     cObjectPtr<IMediaTypeDescription> m_pDescUltrasonic_Front_Center;
	
	tBool m_bDriver, timeflag, setstateready, setstatecomplete, completion, first_sample, first_sample_ready;
	tTimeStamp start_time,end_time,time_diff;
	tBufferID m_szIDDriverI8, m_szIDDriverI16;
	cFilename m_maneuverListFile;
	tFloat32 nextManeuver; 
	tInt16 maneuverCounter;

public:
	cdriverfilter (const tChar* __info);
    virtual ~cdriverfilter();
    
    tResult loadManeuverList();
    tResult process(tTimeStamp start_time, tInt8 action_id, tInt16 maneuver_id);
    /*! this is the list with all the loaded sections from the maneuver list*/
    std::vector<tSector> m_sectorList;
    /*! this is the list with all the loaded MANEUVERS from the maneuver list IN A ROW*/
    std::vector<tAADC_Maneuver> allTheManeuversInARow;
    //tResult PropertyChanged(const char* strProperty);

protected: // overwrites cFilter
    tResult Init(tInitStage eStage, __exception);
    tResult Start(__exception = NULL);
    tResult Stop(__exception = NULL);
    tResult Shutdown(tInitStage eStage, __exception);        
    tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);

private:    
    tBool var_Emergency_Brake_Flag;
    tFloat32 var_accelerate; // output which controls the ardino
    tResult generateop(tUInt32 timeStamp);
};

#endif

