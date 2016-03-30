#include "stdafx.h"
#include "adaptivecruisecontrol.h"
//#include "map.h"

#define timestep 0.05

ADTF_FILTER_PLUGIN("AADC Adaptive Crusie Control", OID_ADTF_ADAPTIVECRUISECONTROL, cadaptivecruisecontrol)
static float  a[] = {0.49, 0.77, 1.0, 1.33, 1.64, 1.87, 2.15, 2.35, 2.48, 2.66, 2.98};

cadaptivecruisecontrol::cadaptivecruisecontrol(const tChar* __info):cFilter(__info)
{
    SetPropertyInt("set_distance", 2);
    SetPropertyBool("set_distance" NSSUBPROP_ISCHANGEABLE, tTrue);

	ultrasonic = 4;
	accaccel = 90.0;
	accactiveflag = tFalse;
	velocity = 0.0f;
	    lookup[0.49]=85;
		lookup[0.77]=84;
		lookup[1.0]=83;
		lookup[1.33]=82;
		lookup[1.64]=81;
		lookup[1.87]=80;
		lookup[2.15]=79;
		lookup[2.35]=78;
		lookup[2.48]=77;
		lookup[2.66]=76;
		lookup[2.98]=75;
LOG_INFO(cString::Format("lookup[1.87]=%f", lookup[1.87]));
}

cadaptivecruisecontrol::~cadaptivecruisecontrol()
{

}
tResult cadaptivecruisecontrol::Init(tInitStage eStage, __exception)
{

	RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))


		if (eStage == StageFirst)
		{

			// create description manager
			cObjectPtr<IMediaDescriptionManager> pDescManager;
			RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**)&pDescManager, __exception_ptr));

			// get media type for input pins
			tChar const * strDescSignalultrasonicfrontmid = pDescManager->GetMediaDescription("tSignalValue");
			RETURN_IF_POINTER_NULL(strDescSignalultrasonicfrontmid);
			cObjectPtr<IMediaType> pTypeSignalultrasonicfrontmid = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalultrasonicfrontmid, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
			RETURN_IF_FAILED(pTypeSignalultrasonicfrontmid->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescultrasonicfrontmid));
			RETURN_IF_FAILED(m_ultrasonicfrontmid.Create("Ultrasonic_Front_Mid", pTypeSignalultrasonicfrontmid, static_cast<IPinEventSink*> (this)));
			RETURN_IF_FAILED(RegisterPin(&m_ultrasonicfrontmid));
			
			tChar const * strDescSignalvelocity = pDescManager->GetMediaDescription("tSignalValue"); 
			RETURN_IF_POINTER_NULL(strDescSignalvelocity);
			cObjectPtr<IMediaType> pTypeSignalvelocity = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalvelocity, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
			RETURN_IF_FAILED(pTypeSignalvelocity->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescvelocity));
			RETURN_IF_FAILED(m_velocity.Create("Velocity", pTypeSignalvelocity, static_cast<IPinEventSink*> (this)));
			RETURN_IF_FAILED(RegisterPin(&m_velocity));
			
			// output pins 
			tChar const * strDescSignalaccaccel = pDescManager->GetMediaDescription("tSignalValue"); //tBoolSignalValue
			RETURN_IF_POINTER_NULL(strDescSignalaccaccel);
			cObjectPtr<IMediaType> pTypeSignalaccaccel = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalaccaccel, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
			RETURN_IF_FAILED(pTypeSignalaccaccel->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescaccaccel));
			RETURN_IF_FAILED(m_accaccel.Create("acc_accel", pTypeSignalaccaccel, static_cast<IPinEventSink*> (this)));
			RETURN_IF_FAILED(RegisterPin(&m_accaccel));

			tChar const * strDescSignalaccactiveflag = pDescManager->GetMediaDescription("tBoolSignalValue"); //tBoolSignalValue
			RETURN_IF_POINTER_NULL(strDescSignalaccactiveflag);
			cObjectPtr<IMediaType> pTypeSignalaccactiveflag = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalaccactiveflag, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
			RETURN_IF_FAILED(pTypeSignalaccactiveflag->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescaccactiveflag));
			RETURN_IF_FAILED(m_accactiveflag.Create("Acc_Active_Flag", pTypeSignalaccactiveflag, static_cast<IPinEventSink*> (this)));
			RETURN_IF_FAILED(RegisterPin(&m_accactiveflag));
		}

		else if (eStage == StageNormal)
		{	

		}
		else if (eStage == StageGraphReady)
		{

		}

		RETURN_NOERROR;

}

tResult cadaptivecruisecontrol::Start(__exception)
		{
			return cFilter::Start(__exception_ptr);
		}

tResult cadaptivecruisecontrol::Stop(__exception)
		{
			return cFilter::Stop(__exception_ptr);
		}

tResult cadaptivecruisecontrol::Shutdown(tInitStage eStage, __exception)
		{
			if (eStage == StageNormal)
			{

			}
			return cFilter::Shutdown(eStage, __exception_ptr);
		}



/*tResult cadaptivecruisecontrol::PropertyChanged(const char* strProperty)
		{
			ReadProperties(strProperty);
			RETURN_NOERROR;
		}
*/
tResult cadaptivecruisecontrol::PropertyChanged(const char* strProperty)
{
	
	set_distance = GetPropertyFloat("set_distance");
	
	RETURN_NOERROR;
}
		
/*tResult cadaptivecruisecontrol::ReadProperties(const tChar* strPropertyName)
		{
			if (NULL == strPropertyName || cString::IsEqual(strPropertyName, S_SET_DISTANCE))
			{
				set_distance = static_cast<tFloat32> (GetPropertyFloat(S_SET_DISTANCE));
			}

			RETURN_NOERROR;
		}*/
		
tResult cadaptivecruisecontrol::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{

	RETURN_IF_POINTER_NULL(pMediaSample);
	RETURN_IF_POINTER_NULL(pSource);


	if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
	{
 
		tUInt32 timeStamp = 0;
		if (pSource == &m_ultrasonicfrontmid)
		{
			last_distance = ultrasonic;
			last_vel = velocity;
			cObjectPtr<IMediaCoder> pCoderInput;
			RETURN_IF_FAILED(m_pDescultrasonicfrontmid->Lock(pMediaSample, &pCoderInput));
			pCoderInput->Get("f32Value", (tVoid*)&ultrasonic);
			pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
			m_pDescultrasonicfrontmid->Unlock(pCoderInput);
			current_distance = ultrasonic;
			acc_calculation(timeStamp); 
			
		}
		
		else if (pSource == &m_velocity)
		{
			cObjectPtr<IMediaCoder> pCoderInput;
			RETURN_IF_FAILED(m_pDescvelocity->Lock(pMediaSample, &pCoderInput));
			pCoderInput->Get("f32Value", (tVoid*)&velocity);
			pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
			m_pDescvelocity->Unlock(pCoderInput);	

		}		
		else 
		{
			RETURN_ERROR(ERR_FAILED);
		}

	}
	RETURN_NOERROR;
}
tResult cadaptivecruisecontrol::acc_calculation(tUInt32 timeStamp)
{
	if(last_distance<set_distance)
	{
		expected_dist = last_distance-last_vel*timestep;
		if(current_distance>expected_dist)
		{
			des_speed = (current_distance-expected_dist)/timestep;
		  if(des_speed<velocity)
                  {
		        for (int i=0;i<10;i++)
		        {
			if (a[i]<des_speed && a[i+1]>des_speed)
			      {
					req_speed=a[i];
					break;
			      }
		        }
                   }
				
		}

		accaccel=lookup[req_speed];
                generateop(timeStamp);
		
	}		
	RETURN_NOERROR;
}



tResult cadaptivecruisecontrol::generateop(tUInt32 timeStamp)
{
	//create new media sample
	cObjectPtr<IMediaSample> pMediaSampleaccaccel;
	cObjectPtr<IMediaSample> pMediaSampleaccactiveflag;
	
	AllocMediaSample((tVoid**)&pMediaSampleaccaccel);
	AllocMediaSample((tVoid**)&pMediaSampleaccactiveflag);

    cObjectPtr<IMediaSerializer> pSerializeraccaccel;
    m_pDescaccaccel->GetMediaSampleSerializer(&pSerializeraccaccel);
	tInt nSizeaccacel = pSerializeraccaccel->GetDeserializedSize();
	pMediaSampleaccaccel->AllocBuffer(nSizeaccacel);
	cObjectPtr<IMediaCoder> pCoderaccaccel;
	m_pDescaccaccel->WriteLock(pMediaSampleaccaccel, &pCoderaccaccel);
	pCoderaccaccel->Set("f32Value", (tVoid*)&(accaccel));
	pCoderaccaccel->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
	m_pDescaccaccel->Unlock(pCoderaccaccel);
	pMediaSampleaccaccel->SetTime(_clock->GetStreamTime());
	m_accaccel.Transmit(pMediaSampleaccaccel);

	cObjectPtr<IMediaSerializer> pSerializeraccactiveflag;
        m_pDescaccactiveflag->GetMediaSampleSerializer(&pSerializeraccactiveflag);
	tInt nSizeaccactiveflag = pSerializeraccactiveflag->GetDeserializedSize();
	pMediaSampleaccactiveflag->AllocBuffer(nSizeaccactiveflag);
	cObjectPtr<IMediaCoder> pCoderaccactiveflag;
	m_pDescaccactiveflag->WriteLock(pMediaSampleaccactiveflag, &pCoderaccactiveflag);
	pCoderaccactiveflag->Set("bValue", (tVoid*)&(accactiveflag));
	pCoderaccactiveflag->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
	m_pDescaccactiveflag->Unlock(pCoderaccactiveflag);
	pMediaSampleaccactiveflag->SetTime(_clock->GetStreamTime());
	m_accactiveflag.Transmit(pMediaSampleaccactiveflag);

	
	RETURN_NOERROR;

}
