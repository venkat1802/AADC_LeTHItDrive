#include "stdafx.h"
#include "lanefollower.h"

using namespace std;
using namespace cv;


ADTF_FILTER_PLUGIN("LaneFollower", OID_ADTF_LaneFollower, cLaneFollower)

cLaneFollower::cLaneFollower(const tChar* __info) : cFilter(__info)
{

	SetPropertyInt("KpsiRIGHT", 26);
	SetPropertyBool("KpsiRIGHT" NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyInt("KpsiLEFT", 24);
	SetPropertyBool("KpsiLEFT" NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyInt("cameraoffset", 100);
	SetPropertyBool("cameraoffset" NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyInt("Ky", 38);
	SetPropertyBool("Ky" NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyInt("vMax", 78);
	SetPropertyBool("vMax" NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyInt("vMin", 78);
	SetPropertyBool("vMin" NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyInt("thresholdvalue", 100);
	SetPropertyBool("thresholdvalue" NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyBool("Enableimshow", tFalse);
	SetPropertyBool("Enableimshow" NSSUBPROP_ISCHANGEABLE, tTrue);

	enable = tFalse; // connect enable to stop output of leTHItdriver filter!!!
	enabled = tFalse;
	freezeThink = tFalse;
	freezeEB = tFalse;
	stopDriver = tFalse;
	freezedThink = tFalse;
	accel = 90;
	steeringAngle = 90;
	steeringAngle_previous = 90;
	mean_theta_previous = 0;
	writeZeroOutputs = tFalse;

}

cLaneFollower::~cLaneFollower()
{
}

tResult cLaneFollower::Init(tInitStage eStage, __exception)
{
	RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

	if (eStage == StageFirst)
	{
		cObjectPtr<IMediaDescriptionManager> pDescManager;
		RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**)&pDescManager, __exception_ptr));

		// Video Input
		RETURN_IF_FAILED(m_oVideoInputPin.Create("Video_Input", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
		RETURN_IF_FAILED(RegisterPin(&m_oVideoInputPin));
		//Enable Input
		tChar const * strDescSignalEnable = pDescManager->GetMediaDescription("tBoolSignalValue");
		RETURN_IF_POINTER_NULL(strDescSignalEnable);
		cObjectPtr<IMediaType> pTypeSignalEnable = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalEnable, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(pTypeSignalEnable->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalInputEnable));
		RETURN_IF_FAILED(m_oEnable.Create("enable", pTypeSignalEnable, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oEnable));
		// Freeze EB input
		tChar const * strDescSignalFreezeEB = pDescManager->GetMediaDescription("tBoolSignalValue");
		RETURN_IF_POINTER_NULL(strDescSignalFreezeEB);
		cObjectPtr<IMediaType> pTypeSignalFreezeEB = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalFreezeEB, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(pTypeSignalFreezeEB->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalInputFreezeEB));
		RETURN_IF_FAILED(m_oFreezeEB.Create("freeze_EB", pTypeSignalFreezeEB, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oFreezeEB));
		// Freeze THINK input
		tChar const * strDescSignalFreezeThink = pDescManager->GetMediaDescription("tBoolSignalValue");
		RETURN_IF_POINTER_NULL(strDescSignalFreezeThink);
		cObjectPtr<IMediaType> pTypeSignalFreezeThink = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalFreezeThink, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(pTypeSignalFreezeThink->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalInputFreezeThink));
		RETURN_IF_FAILED(m_oFreezeThink.Create("freeze_THINK", pTypeSignalFreezeThink, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oFreezeThink));
		// Stop input
		tChar const * strDescSignalStop = pDescManager->GetMediaDescription("tBoolSignalValue");
		RETURN_IF_POINTER_NULL(strDescSignalStop);
		cObjectPtr<IMediaType> pTypeSignalStop = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalStop, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(pTypeSignalStop->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalInputStop));
		RETURN_IF_FAILED(m_oStop.Create("stop_DRIVER", pTypeSignalStop, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oStop));

		// steering output
		tChar const * strDescSignalValueOutputSteer = pDescManager->GetMediaDescription("tSignalValue");
		RETURN_IF_POINTER_NULL(strDescSignalValueOutputSteer);
		cObjectPtr<IMediaType> pTypeSignalValueOutputSteer = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValueOutputSteer, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(pTypeSignalValueOutputSteer->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalOutputSteer));
		RETURN_IF_FAILED(m_oSteer.Create("steering_angle", pTypeSignalValueOutputSteer, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oSteer));
		// acceleration output
		tChar const * strDescSignalValueOutputAccel = pDescManager->GetMediaDescription("tSignalValue");
		RETURN_IF_POINTER_NULL(strDescSignalValueOutputAccel);
		cObjectPtr<IMediaType> pTypeSignalValueOutputAccel = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValueOutputAccel, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(pTypeSignalValueOutputAccel->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalOutputAccel));
		RETURN_IF_FAILED(m_oAccelerate.Create("acceleration", pTypeSignalValueOutputAccel, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oAccelerate));

	}
	else if (eStage == StageNormal)
	{
		firstFrame = tTrue;
		imagecount = 0;

		KpsiRIGHT = GetPropertyInt("KpsiRIGHT");
		KpsiLEFT = GetPropertyInt("KpsiLEFT");
		Ky = GetPropertyInt("Ky");
		vMax = GetPropertyInt("vMax");
		vMin = GetPropertyInt("vMin");
		imshow_enable = GetPropertyBool("Enableimshow");
		thresholdvalue = GetPropertyInt("thresholdvalue");
	}
	RETURN_NOERROR;
}

tResult cLaneFollower::PropertyChanged(const char* strProperty)
{
	KpsiRIGHT = GetPropertyInt("KpsiRIGHT");
	KpsiLEFT = GetPropertyInt("KpsiLEFT");
	Ky = GetPropertyInt("Ky");
	vMax = GetPropertyInt("vMax");
	vMin = GetPropertyInt("vMin");
	cameraoffset = GetPropertyInt("cameraoffset");

	RETURN_NOERROR;
}

tResult cLaneFollower::Shutdown(tInitStage eStage, ucom::IException** __exception_ptr)
{
	switch (eStage)
	{
	case cFilter::StageFirst:
	{
								break;
	}
	default:
		break;
	}
	return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult cLaneFollower::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{
	RETURN_IF_POINTER_NULL(pMediaSample);
	RETURN_IF_POINTER_NULL(pSource);
	if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
	{

		tUInt32 timeStamp = 0;

		if (pSource == &m_oEnable) {
			// read-out the incoming Media Sample
			//get values from media sample
			cObjectPtr<IMediaCoder> pCoderInput;
			RETURN_IF_FAILED(m_pCoderDescSignalInputEnable->Lock(pMediaSample, &pCoderInput));
			pCoderInput->Get("bValue", (tVoid*)&enable);
			pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
			m_pCoderDescSignalInputEnable->Unlock(pCoderInput);
		}
		else if (pSource == &m_oFreezeThink) {
			// read-out the incoming Media Sample
			//get values from media sample
			cObjectPtr<IMediaCoder> pCoderInput;
			RETURN_IF_FAILED(m_pCoderDescSignalInputFreezeThink->Lock(pMediaSample, &pCoderInput));
			pCoderInput->Get("bValue", (tVoid*)&freezeThink);
			pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
			m_pCoderDescSignalInputFreezeThink->Unlock(pCoderInput);
		}
		else if (pSource == &m_oFreezeEB) {
			// read-out the incoming Media Sample
			//get values from media sample
			cObjectPtr<IMediaCoder> pCoderInput;
			RETURN_IF_FAILED(m_pCoderDescSignalInputFreezeEB->Lock(pMediaSample, &pCoderInput));
			pCoderInput->Get("bValue", (tVoid*)&freezeEB);
			pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
			m_pCoderDescSignalInputFreezeEB->Unlock(pCoderInput);
		}
		else if (pSource == &m_oStop) {
			// read-out the incoming Media Sample
			//get values from media sample
			cObjectPtr<IMediaCoder> pCoderInput;
			RETURN_IF_FAILED(m_pCoderDescSignalInputStop->Lock(pMediaSample, &pCoderInput));
			pCoderInput->Get("bValue", (tVoid*)&stopDriver);
			pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
			m_pCoderDescSignalInputStop->Unlock(pCoderInput);
		}
		else if (pSource == &m_oVideoInputPin)
		{
			//Videoformat
			if (firstFrame)
			{
				cObjectPtr<IMediaType> pType;
				RETURN_IF_FAILED(m_oVideoInputPin.GetMediaType(&pType));
				cObjectPtr<IMediaTypeVideo> pTypeVideo;
				RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));
				const tBitmapFormat* pFormat = pTypeVideo->GetFormat();
				if (pFormat == NULL)
				{
					LOG_ERROR("Spurerkennung: No Bitmap information found on pin \"input\"");
					RETURN_ERROR(ERR_NOT_SUPPORTED);
				}
				m_sInputFormat.nPixelFormat = pFormat->nPixelFormat;
				m_sInputFormat.nWidth = pFormat->nWidth;
				m_sInputFormat.nHeight = pFormat->nHeight;
				m_sInputFormat.nBitsPerPixel = pFormat->nBitsPerPixel;
				m_sInputFormat.nBytesPerLine = pFormat->nBytesPerLine;
				m_sInputFormat.nSize = pFormat->nSize;
				m_sInputFormat.nPaletteSize = pFormat->nPaletteSize;
				firstFrame = tFalse;

			}

			// START MODIFIED PART
			if (enable){
				if (!enabled)
					enabled = tTrue;
				freezedThink = tFalse;
			}

			if (freezeThink){
				if (!freezedThink)
					freezedThink = tTrue;
				enabled = tFalse;
			}

			if (enabled && !freezedThink && !freezeEB && !stopDriver){
				steeringAngle = evaluateSteeringAngle(pMediaSample);
				accel = vMax;
				timeStamp = _clock->GetStreamTime() / 1000;
				writeOutputs(timeStamp);
			}

			if (stopDriver){
				enabled = tFalse;
				freezedThink = tFalse; // unuseful if the jury modul is used correctly
				steeringAngle = 90.f;
				accel = 90.f;
				timeStamp = _clock->GetStreamTime() / 1000;
				writeOutputs(timeStamp);
			}

			if (freezeEB){

			}

			if (freezedThink){
				enabled = tFalse;                
			}
			// END MODIFIED PART

			/*
			// START OLD PART
			if (enable){
				freezedThink = tFalse;
				//writeZeroOutputs = tFalse;
			}

			if (enable || enabled){

				if (!enabled){
					enabled = tTrue;
				}

				if (freezeEB){

				}
				else if (freezeThink || freezedThink){
					//enabled = tFalse;
					accel = 0.f;
					steeringAngle = 0.f;
					freezedThink = tTrue;
					//if (!writeZeroOutputs){
					//	writeOutputs(timeStamp);
					//	writeZeroOutputs = tTrue;
					//}

				}
				else if (stopDriver){
					freezedThink = tFalse;
					accel = 0.f;
					steeringAngle = 0.f;
				}
				else{
					//writeZeroOutputs = tFalse;
					freezedThink = tFalse;
					steeringAngle = evaluateSteeringAngle(pMediaSample);
					accel = vMax; // - abs(steeringAngle)*(vMax-vMin) / 35;                                        
				}

			}

			if ((enable || enabled) && (!freezeThink && !freezedThink) && !freezeEB){ //freeze from THINK, freeze from EB (if stop from JURY then write 0 outputs)
				timeStamp = _clock->GetStreamTime() / 1000;
				writeOutputs(timeStamp);
			}
			// END OLD PART
			*/

		}

	}
	if (nEventCode == IPinEventSink::PE_MediaTypeChanged)
	{
		if (pSource == &m_oVideoInputPin)
		{
			cObjectPtr<IMediaType> pType;
			RETURN_IF_FAILED(m_oVideoInputPin.GetMediaType(&pType));
			cObjectPtr<IMediaTypeVideo> pTypeVideo;
			RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));
			UpdateImageFormat(m_oVideoInputPin.GetFormat());
		}
	}

	RETURN_NOERROR;
}

// implementation
tFloat32 cLaneFollower::evaluateSteeringAngle(IMediaSample* pSample)
{
	// VideoInput
	RETURN_IF_POINTER_NULL(pSample);

	cObjectPtr<IMediaSample> pNewRGBSample;

	const tVoid* l_pSrcBuffer;

	if (IS_OK(pSample->Lock(&l_pSrcBuffer)))
	{

		IplImage* img = cvCreateImageHeader(cvSize(m_sInputFormat.nWidth, m_sInputFormat.nHeight), IPL_DEPTH_8U, 3);
		img->imageData = (char*)l_pSrcBuffer;
		//Übergang von OpenCV1 auf OpenCV2
		Mat image(cvarrToMat(img));
		cvReleaseImage(&img);
		pSample->Unlock(l_pSrcBuffer);
		//Zuschneiden des Bildes
		Mat imagecut;
		imagecut = image(Range(260, 400), Range(120,600)).clone();
		//Erzeugen eines Graustufenbildes
		cvtColor(imagecut, grey, CV_BGR2GRAY);
		//Treshold um Binärbild zu erhalten
		threshold(grey, greythresh, thresholdvalue, 500, THRESH_BINARY);
		//Kantendedektion
		Canny(greythresh, linecanny, 0, 2, 3, tFalse);
		if(imshow_enable)
		{
		imshow("cannyimage",linecanny);
		imshow("greyimage",greythresh);
		waitKey(1);
		}
		cannysize = linecanny.size();
		vector<Vec2f> lines;
		HoughLines(linecanny, lines, 2, CV_PI / 180, 75, 0, 0);

		tFloat32 thetaAll[1000];
		tFloat32 rhoAll[1000];
		for (tInt i = 0; i < (tInt)(lines.size()); i++)
		{
			thetaAll[i] = lines[i][1];
			rhoAll[i] = lines[i][0];
		}

		tFloat32 thetaNoRep[1000];
		tFloat32 rhoNoRep[1000];
		tInt sizeNoRep = 0;
		for (tInt i = 0; i < (tInt)(lines.size()); i++)
		{
			tInt rep = 0;
			for (tInt j = 0; j < i; j++)
			{
				if (abs(thetaAll[i] - thetaAll[j]) <= 0.2)
				{
					rep = 1;
				}
			}
			if ((rep == 0) && (thetaAll[i]<CV_PI*0.40 || thetaAll[i]>CV_PI*0.60))
			{
				thetaNoRep[sizeNoRep] = thetaAll[i];
				rhoNoRep[sizeNoRep] = rhoAll[i];
				sizeNoRep += 1;
			}
		}

		tFloat32 mean_theta = 0;
		tFloat32 x1, x2, deltaR, deltaL, deltaRL = 0;
		for (tInt i = 0; i < sizeNoRep; i++)
		{
			if (thetaNoRep[i] >= CV_PI / 2)
				mean_theta += thetaNoRep[i] - (tFloat32)CV_PI;
			else
				mean_theta += thetaNoRep[i];
		}

		if (sizeNoRep >= 2){
			x1 = rhoNoRep[0] * cos(thetaNoRep[0]) - (240 - rhoNoRep[0] * sin(thetaNoRep[0]))*tan(thetaNoRep[0]);
			x2 = rhoNoRep[1] * cos(thetaNoRep[1]) - (240 - rhoNoRep[1] * sin(thetaNoRep[1]))*tan(thetaNoRep[1]);
			if (x1 > x2){
				deltaR = x1 - 320;
				deltaL = 320 - x2;
			}
			else{
				deltaR = x2 - 320;
				deltaL = 320 - x1;
			}
			deltaRL = deltaR - deltaL + cameraoffset;
		}
		if (sizeNoRep == 1){
			x1 = rhoNoRep[0] * cos(thetaNoRep[0]) - (240 - rhoNoRep[0] * sin(thetaNoRep[0]))*tan(thetaNoRep[0]);
			if (x1 > 320){
				deltaRL = x1 - 640;
			}
			else{
				deltaRL = x1; // deltaRL = x1 - 0;
			}
		}

		if (abs(deltaRL * Ky / 1000) >= 25)
			deltaRL = 0;

		if ((sizeNoRep != 0) && (sizeNoRep != 1)){
			mean_theta /= sizeNoRep;
			if (mean_theta > 0)
				steeringAngle = KpsiRIGHT * mean_theta + Ky / 1000 * deltaRL;
			else
				steeringAngle = KpsiLEFT * mean_theta + Ky / 1000 * deltaRL;
		}
		else if (sizeNoRep == 1)
		if (mean_theta > 0)
			steeringAngle = KpsiRIGHT * mean_theta / 2 + Ky / 1000 * deltaRL;
		else
			steeringAngle = KpsiLEFT * mean_theta / 2 + Ky / 1000 * deltaRL;
		else
			steeringAngle = steeringAngle_previous;

		steeringAngle_previous = steeringAngle;
		mean_theta_previous = mean_theta;

	}

	this->steeringAng = 90.0;
	
	if (steeringAngle != 0)
		this->steeringAng = (steeringAngle*0.856) + 90;		// after 90 is -ve, then 0.856 refers to 60/70
	
	// min steeringAngle = 60, max = 120, prev -35 to +35
	if (this->steeringAng > 120)
		this->steeringAng = 120;
	if (this->steeringAng < 60)
		this->steeringAng = 60;

	return this->steeringAng;

}

tResult cLaneFollower::UpdateImageFormat(const tBitmapFormat* pFormat)
{
	if (pFormat != NULL)
	{
		m_sInputFormat = (*pFormat);
		LOG_INFO(adtf_util::cString::Format("Spurerkennung Filter Input: Size %d x %d ; BPL %d ; Size %d , PixelFormat; %d", m_sInputFormat.nWidth, m_sInputFormat.nHeight,
			m_sInputFormat.nBytesPerLine, m_sInputFormat.nSize, m_sInputFormat.nPixelFormat));
	}

	RETURN_NOERROR;
}

tResult cLaneFollower::writeOutputs(tUInt32 timeStamp)
{

	//create new media sample
	cObjectPtr<IMediaSample> pMediaSampleAccel;
	AllocMediaSample((tVoid**)&pMediaSampleAccel);
	//allocate memory with the size given by the descriptor
	cObjectPtr<IMediaSerializer> pSerializerAccel;
	m_pCoderDescSignalOutputAccel->GetMediaSampleSerializer(&pSerializerAccel);
	tInt nSizeAccel = pSerializerAccel->GetDeserializedSize();
	pMediaSampleAccel->AllocBuffer(nSizeAccel);
	//write date to the media sample with the coder of the descriptor
	cObjectPtr<IMediaCoder> pCoderOutputAccel;
	m_pCoderDescSignalOutputAccel->WriteLock(pMediaSampleAccel, &pCoderOutputAccel);
	// ...		
	pCoderOutputAccel->Set("f32Value", (tVoid*)&(accel));
	pCoderOutputAccel->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
	m_pCoderDescSignalOutputAccel->Unlock(pCoderOutputAccel);
	//transmit media sample over output pin
	pMediaSampleAccel->SetTime(_clock->GetStreamTime());
	m_oAccelerate.Transmit(pMediaSampleAccel);

	//create new media sample
	cObjectPtr<IMediaSample> pMediaSampleSteer;
	AllocMediaSample((tVoid**)&pMediaSampleSteer);
	//allocate memory with the size given by the descriptor
	cObjectPtr<IMediaSerializer> pSerializerSteer;
	m_pCoderDescSignalOutputSteer->GetMediaSampleSerializer(&pSerializerSteer);
	tInt nSizeSteer = pSerializerSteer->GetDeserializedSize();
	pMediaSampleSteer->AllocBuffer(nSizeSteer);
	//write date to the media sample with the coder of the descriptor
	cObjectPtr<IMediaCoder> pCoderOutputSteer;
	m_pCoderDescSignalOutputSteer->WriteLock(pMediaSampleSteer, &pCoderOutputSteer);
	// ...
	pCoderOutputSteer->Set("f32Value", (tVoid*)&(steeringAngle));
	pCoderOutputSteer->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
	m_pCoderDescSignalOutputSteer->Unlock(pCoderOutputSteer);
	//transmit media sample over output pin
	pMediaSampleSteer->SetTime(_clock->GetStreamTime());
	m_oSteer.Transmit(pMediaSampleSteer);

	RETURN_NOERROR;

}
