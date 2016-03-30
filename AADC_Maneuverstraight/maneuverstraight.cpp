#include "stdafx.h"
//#include <cmath>
#include "maneuverstraight.h"

#define DISTANCE_TRAJ1_STRAIGHT "cmaneuverstraight::distance_st1"
#define DISTANCE_TRAJ2_STRAIGHT "cmaneuverstraight::distance_st2"

#define MANEUVER_INTERSECTION_STRAIGHT   (tFloat32)7.0
#define MANEUVER_INTERSECTION_DEFAULT   (tFloat32)10.0

#define FLT_EPSILONA (tFloat32)0.01
//#define FLT_EPSILON (tFloat32)0.01

ADTF_FILTER_PLUGIN("AADC Maneuver straight", OID_ADTF_MANEUVERSTRAIGHT, cmaneuverstraight)

cmaneuverstraight::cmaneuverstraight(const tChar* __info):cFilter(__info)
{

	SetPropertyFloat(DISTANCE_TRAJ1_STRAIGHT,0.3);
    SetPropertyBool(DISTANCE_TRAJ1_STRAIGHT NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(DISTANCE_TRAJ1_STRAIGHT NSSUBPROP_DESCRIPTION, "the distance for the 1st trajectory for straight");

	SetPropertyFloat(DISTANCE_TRAJ2_STRAIGHT,1);
    SetPropertyBool(DISTANCE_TRAJ2_STRAIGHT NSSUBPROP_ISCHANGEABLE,tTrue);
    SetPropertyStr(DISTANCE_TRAJ2_STRAIGHT NSSUBPROP_DESCRIPTION, "the distance for the 2nd straight");

    dist_at_start = 0;
	start= MANEUVER_INTERSECTION_DEFAULT;
	first_sample= tFalse;
	finishflag = tFalse;
	ReadProperties(NULL);
}

cmaneuverstraight::~cmaneuverstraight()
{

}
tResult cmaneuverstraight::Init(tInitStage eStage, __exception)
{

	RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))


		if (eStage == StageFirst)
		{

			// create description manager
			cObjectPtr<IMediaDescriptionManager> pDescManager;
			RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**)&pDescManager, __exception_ptr));

			// get media type for input pins
			tChar const * strDescSignaldistanceoverall = pDescManager->GetMediaDescription("tSignalValue");
			RETURN_IF_POINTER_NULL(strDescSignaldistanceoverall);
			cObjectPtr<IMediaType> pTypeSignaldistanceoverall = new cMediaType(0, 0, 0, "tSignalValue", strDescSignaldistanceoverall, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
			RETURN_IF_FAILED(pTypeSignaldistanceoverall->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescdistanceoverall));
			RETURN_IF_FAILED(m_distanceoverall.Create("Distance_OverallS", pTypeSignaldistanceoverall, static_cast<IPinEventSink*> (this)));
			RETURN_IF_FAILED(RegisterPin(&m_distanceoverall));

			tChar const * strDescSignalstart = pDescManager->GetMediaDescription("tSignalValue"); //tBoolSignalValue
			RETURN_IF_POINTER_NULL(strDescSignalstart);
			cObjectPtr<IMediaType> pTypeSignalstart = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalstart, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
			RETURN_IF_FAILED(pTypeSignalstart->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescstart));
			RETURN_IF_FAILED(m_start.Create("startS", pTypeSignalstart, static_cast<IPinEventSink*> (this)));
			RETURN_IF_FAILED(RegisterPin(&m_start));

			tChar const * strDescSignalfinishflag = pDescManager->GetMediaDescription("tBoolSignalValue"); //tBoolSignalValue
			RETURN_IF_POINTER_NULL(strDescSignalfinishflag);
			cObjectPtr<IMediaType> pTypeSignalfinishflag = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalfinishflag, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
			RETURN_IF_FAILED(pTypeSignalfinishflag->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescfinishflag));
			RETURN_IF_FAILED(m_ofinishflag.Create("FinishFlagS", pTypeSignalfinishflag, static_cast<IPinEventSink*> (this)));
			RETURN_IF_FAILED(RegisterPin(&m_ofinishflag));

		}

		else if (eStage == StageNormal)
		{
			

		}
		else if (eStage == StageGraphReady)
		{

		}

		RETURN_NOERROR;

}

tResult cmaneuverstraight::Start(__exception)
		{
			return cFilter::Start(__exception_ptr);
		}

tResult cmaneuverstraight::Stop(__exception)
		{
			return cFilter::Stop(__exception_ptr);
		}

tResult cmaneuverstraight::Shutdown(tInitStage eStage, __exception)
		{
			if (eStage == StageNormal)
			{

			}
			return cFilter::Shutdown(eStage, __exception_ptr);
		}



tResult cmaneuverstraight::PropertyChanged(const char* strProperty)
		{
			ReadProperties(strProperty);
			RETURN_NOERROR;
		}
		
tResult cmaneuverstraight::ReadProperties(const tChar* strPropertyName)
		{
			if (NULL == strPropertyName || cString::IsEqual(strPropertyName, DISTANCE_TRAJ1_STRAIGHT))
			{
				distance_st1_straight = static_cast<tFloat32> (GetPropertyFloat(DISTANCE_TRAJ1_STRAIGHT));
			}

			if (NULL == strPropertyName || cString::IsEqual(strPropertyName, DISTANCE_TRAJ2_STRAIGHT))
			{
				distance_st1_straight_afterT = static_cast<tFloat32> (GetPropertyFloat(DISTANCE_TRAJ2_STRAIGHT));
			}

			RETURN_NOERROR;
		}
		
tResult cmaneuverstraight::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
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
			if((fabs(MANEUVER_INTERSECTION_STRAIGHT - start) < FLT_EPSILON) && !first_sample)
			{
				first_sample=tTrue;
                dist_at_start = distanceoverall;
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
            if((fabs(MANEUVER_INTERSECTION_STRAIGHT - start) < FLT_EPSILON))
            	maneuver_straight(timeStamp);
            else
            	{}//LOG_INFO(cString::Format("Maneuver Right intresection maneuver is started, start=%f", start));
	    }
				
		else 
		{
			RETURN_ERROR(ERR_FAILED);
		}

	}
	RETURN_NOERROR;
}


tResult cmaneuverstraight::maneuver_straight(tUInt32 timeStamp)
{
if (first_sample==tTrue)
{
   finishflag = tFalse;
   if (distanceoverall > (dist_at_start+distance_st1_straight+distance_st1_straight_afterT))
   {
	   	finishflag = tTrue;
        first_sample= tFalse;
        start = MANEUVER_INTERSECTION_DEFAULT;
        LOG_INFO(cString::Format("maneuver straight is completed, finishflag=%d", finishflag));
	}
generateop(timeStamp);
}
	RETURN_NOERROR;
}

tResult cmaneuverstraight::generateop(tUInt32 timeStamp)
{
	cObjectPtr<IMediaSample> pMediaSamplefinishflag;
	AllocMediaSample((tVoid**)&pMediaSamplefinishflag);

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
	RETURN_NOERROR;
}
