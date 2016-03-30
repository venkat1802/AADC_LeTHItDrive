#pragma once

#ifndef _SPEEDCONTROLLER_H_
#define _SPEEDCONTROLLER_H_

#include "stdafx.h"

#define OID_ADTF_SPEEDCONTROLLER "adtf.aadc.cspeedcontroller"

#define default_starting_phase (tFloat32)50.0

#define pullout_parallel_active (tFloat32)1.0
#define pulloutleft_active (tFloat32)2.0
#define pulloutright_active (tFloat32)3.0
#define pullout_finished (tFloat32)4.0

#define parallelparking_active (tFloat32)15.0
#define crossparking_active (tFloat32)16.0
#define parking_deactive (tFloat32)17.0

#define overtaking_active (tFloat32)20.0
#define overtaking_deactive (tFloat32)21.0

#define Maneuvers_int_active (tFloat32)25.0
#define Maneuvers_int_deactive (tFloat32)26.0

#define Lane_Follower_Active (tFloat32)30.0

enum SpeedControllerstatusflag
{
	NO_Maneuver_Active,
	Pullover_Active,
	LaneFollower_Active,
	Maneuvers_Active,
	Parking_Active,
	Overtaking_Active
};

class cspeedcontroller : public adtf::cFilter
{
    //ADTF_DECLARE_FILTER_VERSION(OID_ADTF_SPEEDCONTROLLER, "AADC SPEED CONTROLLER ", OBJCAT_DataFilter);    
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_SPEEDCONTROLLER, "AADC SPEED CONTROLLER ", OBJCAT_DataFilter, "Speed Controller", 1, 0,0, "Beta Version");    

    cInputPin Emergency_Brake_Flag;                
    cInputPin pullout_acc;
	cInputPin pullout_steer;
    cInputPin Lanefollower_Acc;
	cInputPin Lanefollower_steer;
	cInputPin maneuver_accel1;
	cInputPin maneuver_steer1;
	cInputPin parking_steer;
	cInputPin parking_acc;
	cInputPin ovtaking_steer;
	cInputPin ovtaking_acc;
	//cInputPin Parking_accel;
	//cInputPin Parking_steer;
	cInputPin m_maneuver_statusflag;
    cInputPin m_Ultrasonic_Front_Center;
    cInputPin Signal_IDS;
    cInputPin JuryStart_Stop;
    cInputPin Jurycomplete;
		
    //headlight incase of emergency braking
    cOutputPin m_headlight;
	cOutputPin m_accelerate;
	cOutputPin m_steer;

	cOutputPin current_active_maneuver;
	cOutputPin finishstatus_maneuver;
	cOutputPin Activestatus_maneuver;

    cObjectPtr<IMediaTypeDescription> med_typ_Emergency_Brake_Flag;
	cObjectPtr<IMediaTypeDescription> med_typ_Lanefollower_Acc;
    cObjectPtr<IMediaTypeDescription> med_typ_Lanefollower_steer;
	cObjectPtr<IMediaTypeDescription> med_typ_accel1;
	cObjectPtr<IMediaTypeDescription> med_typ_steer1;
	cObjectPtr<IMediaTypeDescription> med_typ_m_maneuver_start_flag;
    cObjectPtr<IMediaTypeDescription> m_pDescUltrasonic_Front_Center;
	cObjectPtr<IMediaTypeDescription> med_typ_pullout_acc;
    cObjectPtr<IMediaTypeDescription> med_typ_pullout_steer;
	cObjectPtr<IMediaTypeDescription> med_typ_parking_acc;
    cObjectPtr<IMediaTypeDescription> med_typ_parking_steer;
	cObjectPtr<IMediaTypeDescription> med_typ_ovtaking_acc;
    cObjectPtr<IMediaTypeDescription> med_typ_ovtaking_steer;
    cObjectPtr<IMediaTypeDescription> med_typ_Signal_IDS;

	cObjectPtr<IMediaTypeDescription> med_typ_m_accelerate;
	cObjectPtr<IMediaTypeDescription> med_typ_m_steer;
	cObjectPtr<IMediaTypeDescription> med_typ_m_headlight;
	cObjectPtr<IMediaTypeDescription> med_typ_current_active_maneuver;
	cObjectPtr<IMediaTypeDescription> med_typ_finishstatus_maneuver;
	cObjectPtr<IMediaTypeDescription> med_typ_activestatus_maneuver;
	cObjectPtr<IMediaTypeDescription> med_typ_JuryStart_Stop;
	cObjectPtr<IMediaTypeDescription> med_typ_Jurycomplete;

public:
    cspeedcontroller(const tChar* __info);
    virtual ~cspeedcontroller();
    //tResult PropertyChanged(const char* strProperty);

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
    //tFloat32 Ultrasonic_Front_Center;
    SpeedControllerstatusflag Controllerstatusflag;
    tBool var_jurycomplete;
    tBool var_Emergency_Brake_Flag;
    tBool varJury_start_stop;
    tBool Maneuver_Active_Flag;
    tBool var_headlight;
    tFloat32 var_pullout_acc, var_pullout_steer;
    tFloat32 var_Lanefollower_Acc,var_Lanefollower_steer; // from Lane tracker
	tFloat32 var_maneuver_accel1, var_maneuver_steer1; // from maneuver
    tFloat32 var_accelerate, var_steer; // output which controls the ardino
	tBool maneuver_start_flag;
	tFloat32 var_signal_id;
	tFloat32 var_Parking_Acc,var_Parking_steer;
	tFloat32 var_ovtaking_Acc,var_ovtaking_steer;
	tResult generateop(tUInt32 timeStamp);
	tResult generateop_finishstatus(tUInt32 timeStamp);
};

#endif

