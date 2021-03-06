#include "stdafx.h"
#include "PPinit.h"

#define MARKER_ID_UNMARKEDINTERSECTION 0
#define	MARKER_ID_STOPANDGIVEWAY 1
#define	MARKER_ID_PARKINGAREA 2
#define	MARKER_ID_HAVEWAY 3
#define MARKER_ID_AHEADONLY 4
#define	MARKER_ID_GIVEWAY 5
#define	MARKER_ID_PEDESTRIANCROSSING 6
#define MARKER_ID_ROUNDABOUT 7
#define	MARKER_ID_NOOVERTAKING 8
#define	MARKER_ID_NOENTRYVEHICULARTRAFFIC 9
#define	MARKER_ID_ONEWAYSTREET 0

// for property browser
//#define MD_REQUIRED_MARKER_SIZE "ManeuverDetection::Required marker size"
/*#define PP_ROWS_R1 "initPP::ROW1"
#define PP_ROWS_R2 "initPP::ROW2"
#define PP_COLS_C1 "initPP::COL1"
#define PP_COLS_C2 "initPP::COL2"*/
#define PP_ENABLE_IMSHOW  "initPP::Enableimshow"
#define MD_REQUIRED_MARKER_SIZE "initPP::Required marker size"

ADTF_FILTER_PLUGIN("Parallel Park Initialize",OID_ADTF_DEMO_START_PARALLEL_PARK, cParallelParkInit)



cParallelParkInit::cParallelParkInit(const tChar* __info):cFilter(__info)
{   
    parkstart_flag = tFalse;
    maneuverfinish_flag = tFalse;
    direct_flag = tFalse;
    templateFound_flag = tFalse;
    markercheckPass_flag = tFalse;

	indicatorLeft = tFalse;
	indicatorRight = tFalse;
	headLights = tFalse;	
	
	Xdesired = 0;
	Ydesired = 0;

    SetPropertyBool("Debug Output to Console", m_bDebugModeEnabled);
    SetPropertyStr("Debug Output to Console" NSSUBPROP_DESCRIPTION,"If enabled additional debug information is printed to the console");

    SetPropertyBool(PP_ENABLE_IMSHOW,imshowflag);
    SetPropertyStr(PP_ENABLE_IMSHOW NSSUBPROP_DESCRIPTION, "If true imshow will be enabled");

   /* SetPropertyInt(PP_ROWS_R1, 300);
    SetPropertyBool(PP_ROWS_R1 NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(PP_ROWS_R1 NSSUBPROP_DESCRIPTION, "initPP::The first row R1 to be taken (R1,) (R2,)");

    SetPropertyInt(PP_ROWS_R2, 480);
    SetPropertyBool(PP_ROWS_R2 NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(PP_ROWS_R2 NSSUBPROP_DESCRIPTION,"initPP::The second row R2 to be taken (R1,) (R2,)");

    SetPropertyInt(PP_COLS_C1, 350);
    SetPropertyBool(PP_COLS_C1 NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(PP_COLS_C1 NSSUBPROP_DESCRIPTION, "initPP::The first col C1 to be taken (,C1) (,C2)");

    SetPropertyInt(PP_COLS_C2, 640);
    SetPropertyBool(PP_COLS_C2 NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(PP_COLS_C2 NSSUBPROP_DESCRIPTION, "initPP::The second col C2 to be taken (,C1) (,C2)");*/

    SetPropertyFloat(MD_REQUIRED_MARKER_SIZE, 700);
    SetPropertyBool(MD_REQUIRED_MARKER_SIZE NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(MD_REQUIRED_MARKER_SIZE NSSUBPROP_DESCRIPTION, "MarkerDetection::The marker size above which manuevers processing starts");

}

cParallelParkInit::~cParallelParkInit()
{
}

tResult cParallelParkInit::PropertyChanged(const char* strProperty)
{
    ReadProperties(strProperty);
    RETURN_NOERROR;
}

tResult cParallelParkInit::ReadProperties(const tChar* strPropertyName)
{
    
    /*if (NULL == strPropertyName || cString::IsEqual(strPropertyName, PP_ROWS_R1))
    {
        row1 = GetPropertyInt(PP_ROWS_R1);
    }
    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, PP_ROWS_R2))
    {
    	row2 = GetPropertyInt(PP_ROWS_R2);
    }
    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, PP_COLS_C1))
    {
        col1 = GetPropertyInt(PP_COLS_C1);
    }
    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, PP_COLS_C2))
    {
    	col2 = GetPropertyInt(PP_COLS_C2);
    }*/
    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, PP_ENABLE_IMSHOW ))
    {
    	imshowflag = GetPropertyBool(PP_ENABLE_IMSHOW);
    }
    if (NULL == strPropertyName || cString::IsEqual(strPropertyName,"Debug Output to Console"))
    {
    	m_bDebugModeEnabled = GetPropertyBool("Debug Output to Console");
    }
    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, MD_REQUIRED_MARKER_SIZE))
    {
    	f32RequiredArea = GetPropertyFloat(MD_REQUIRED_MARKER_SIZE);
    }

    RETURN_NOERROR;

    
}
tResult cParallelParkInit::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));
    if (eStage == StageFirst)
        {

        //create the video rgb input pin
        RETURN_IF_FAILED(m_oVideoInputPin.Create("Video_RGB_input",IPin::PD_Input, static_cast<IPinEventSink*>(this))); 
        RETURN_IF_FAILED(RegisterPin(&m_oVideoInputPin));

        //create the video rgb output pin
        RETURN_IF_FAILED(m_oVideoOutputPin.Create("Video_RGB_output", IPin::PD_Output, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oVideoOutputPin));

	cObjectPtr<IMediaDescriptionManager> pDescManager;
	RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**)&pDescManager, __exception_ptr));



	/*tChar const * strDescSignalmaneuverright = pDescManager->GetMediaDescription("tBoolSignalValue");
	RETURN_IF_POINTER_NULL(strDescSignalmaneuverright);
	cObjectPtr<IMediaType> pTypeSignalmaneuverright = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalmaneuverright, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
	RETURN_IF_FAILED(pTypeSignalmaneuverright->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pmaneuverright_flag));
	RETURN_IF_FAILED(m_omarkerCheckPass.Create("n", pTypeSignalmaneuverright, static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&m_omarkerCheckPass));

	tChar const * strDescSignalmaneuverleft = pDescManager->GetMediaDescription("tBoolSignalValue");
	RETURN_IF_POINTER_NULL(strDescSignalmaneuverleft);
	cObjectPtr<IMediaType> pTypeSignalmaneuverleft = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalmaneuverleft, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
	RETURN_IF_FAILED(pTypeSignalmaneuverleft->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pmaneuverleft_flag));
	RETURN_IF_FAILED(m_otemplateFoundPass.Create("M", pTypeSignalmaneuverleft, static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&m_otemplateFoundPass));*/

	tChar const * strDescSignalmarkercheckPass = pDescManager->GetMediaDescription("tBoolSignalValue");
	RETURN_IF_POINTER_NULL(strDescSignalmarkercheckPass);
	cObjectPtr<IMediaType> pTypeSignalmarkercheckPass = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalmarkercheckPass, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
	RETURN_IF_FAILED(pTypeSignalmarkercheckPass->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_markercheckPass_flag));
	RETURN_IF_FAILED(m_omarkerCheckPass.Create("Park Marker Found", pTypeSignalmarkercheckPass, static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&m_omarkerCheckPass));

	tChar const * strDescSignaltemplateFoundPass = pDescManager->GetMediaDescription("tBoolSignalValue");
	RETURN_IF_POINTER_NULL(strDescSignaltemplateFoundPass);
	cObjectPtr<IMediaType> pTypeSignaltemplateFoundPass = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignaltemplateFoundPass, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
	RETURN_IF_FAILED(pTypeSignaltemplateFoundPass->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_templateFound_flag));
	RETURN_IF_FAILED(m_otemplateFoundPass.Create("Template Found", pTypeSignaltemplateFoundPass, static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&m_otemplateFoundPass));

	//cObjectPtr<IMediaTypeDescription> pMediaTypeDesc;
        tChar const * strDesc = pDescManager->GetMediaDescription("tRoadSign");   
        RETURN_IF_POINTER_NULL(strDesc);    
        cObjectPtr<IMediaType> pType = new cMediaType(0, 0, 0, "tRoadSign", strDesc, IMediaDescription::MDF_DDL_DEFAULT_VERSION);        
        // create the road sign OutputPin
        RETURN_IF_FAILED(m_signDetect.Create("RoadSign", pType, this));
        RETURN_IF_FAILED(RegisterPin(&m_signDetect));
        // set the description for the road sign pin
        RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionRoadSign));

		//X goal point Output
		tChar const * strDescSignalXgoal = pDescManager->GetMediaDescription("tSignalValue");
		RETURN_IF_POINTER_NULL(strDescSignalXgoal);
		cObjectPtr<IMediaType> pTypeSignalXgoal = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalXgoal, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(pTypeSignalXgoal->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalXgoal));
		RETURN_IF_FAILED(m_oXgoal.Create("Xgoal", pTypeSignalXgoal, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oXgoal));

		//Y goal point Output
		tChar const * strDescSignalYgoal = pDescManager->GetMediaDescription("tSignalValue");
		RETURN_IF_POINTER_NULL(strDescSignalYgoal);
		cObjectPtr<IMediaType> pTypeSignalYgoal = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalYgoal, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(pTypeSignalYgoal->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalYgoal));
		RETURN_IF_FAILED(m_oYgoal.Create("Ygoal", pTypeSignalYgoal, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oYgoal));


	RETURN_NOERROR;




			



        }
    else if (eStage == StageNormal)
        { 
		m_bFirstFrame = tTrue;
		i16ID = 0;
		f32Area = 0.0;
       
		ReadProperties(NULL);
        }
    else if (eStage == StageGraphReady)
        {
	m_bIDsRoadSignSet = tFalse;
        // get the image format of the input video pin
        cObjectPtr<IMediaType> pType;
        RETURN_IF_FAILED(m_oVideoInputPin.GetMediaType(&pType));

        cObjectPtr<IMediaTypeVideo> pTypeVideo;
        RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));
    
        // set the image format of the input video pin
        UpdateInputImageFormat(pTypeVideo->GetFormat());  
    
        // set the image format of the output video pin
        UpdateOutputImageFormat(pTypeVideo->GetFormat());  
    

        }
    RETURN_NOERROR;
}

tResult cParallelParkInit::Start(__exception)
{
	return cFilter::Start(__exception_ptr);
}

tResult cParallelParkInit::Stop(__exception)
{
	return cFilter::Stop(__exception_ptr);
}

tResult cParallelParkInit::Shutdown(tInitStage eStage, __exception)
{
	if (eStage == StageNormal)
	{

	}
	return cFilter::Shutdown(eStage, __exception_ptr);
}
tResult cParallelParkInit::OnPinEvent(IPin* pSource,
                                            tInt nEventCode,
                                            tInt nParam1,
                                            tInt nParam2,
                                            IMediaSample* pMediaSample)
{
    switch (nEventCode)
            {
            case IPinEventSink::PE_MediaSampleReceived:
		timeStamp = 0;
                // a new image was received so the processing is started
		if (pSource == &m_signDetect && m_pDescriptionRoadSign != NULL)
		{
			__adtf_sample_read_lock_mediadescription(m_pDescriptionRoadSign, pMediaSample, pCoderInput);
			if (!m_bIDsRoadSignSet)
			{
				pCoderInput->GetID("i16Identifier",m_szIDRoadSignI16Identifier);
				pCoderInput->GetID("f32Imagesize",m_szIDRoadSignF32Imagesize);
				m_bIDsRoadSignSet = tTrue;
			}
			// get the values from sample
			pCoderInput->Get(m_szIDRoadSignI16Identifier, (tVoid*) &i16ID);
			pCoderInput->Get(m_szIDRoadSignF32Imagesize, (tVoid*) &f32Area);
			if (m_bDebugModeEnabled)
				LOG_INFO(cString::Format("MarkerDetection: Id -> %d Area: %f", i16ID,f32Area));

			if(( MARKER_ID_PARKINGAREA == i16ID ) && (f32Area > f32RequiredArea)) //REQUIRED_AREA)
			{
				enable_video_detection = tTrue;
                                markercheckPass_flag = tTrue;
				direct_flag = tFalse;
                                
			        //if (m_bDebugModeEnabled)
				LOG_INFO(cString::Format("marker1 Detection : Id -> %d with required Area: %f is detected", i16ID,f32Area));
			}
			else
				direct_flag = tTrue;
		}
                else if (markercheckPass_flag && !maneuverfinish_flag && pSource == &m_oVideoInputPin)  // process the video input here
		{
	        	tTimeStamp InputTimeStamp;
	        	InputTimeStamp = pMediaSample->GetTime(); 
  			ProcessVideo(pMediaSample);   
		}                
                break;
            case IPinEventSink::PE_MediaTypeChanged:
                if (pSource == &m_oVideoInputPin)
                {
                    //the input format was changed, so the imageformat has to changed in this filter also
                    cObjectPtr<IMediaType> pType;
                    RETURN_IF_FAILED(m_oVideoInputPin.GetMediaType(&pType));

                    cObjectPtr<IMediaTypeVideo> pTypeVideo;
                    RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));

                    UpdateInputImageFormat(m_oVideoInputPin.GetFormat());    
                    UpdateOutputImageFormat(m_oVideoInputPin.GetFormat());                
                }            
                break;
            default:
                break;
        }      
    RETURN_NOERROR;
}

tResult cParallelParkInit::ProcessVideo(adtf::IMediaSample* pISample)
{


    RETURN_IF_POINTER_NULL(pISample);

    //creating new media sample for output
    cObjectPtr<IMediaSample> pNewSample;
    RETURN_IF_FAILED(_runtime->CreateInstance(OID_ADTF_MEDIA_SAMPLE, IID_ADTF_MEDIA_SAMPLE, (tVoid**) &pNewSample));
    RETURN_IF_FAILED(pNewSample->AllocBuffer(m_sOutputFormat.nSize));    
    
    //creating new pointer for input data
    const tVoid* l_pSrcBuffer;
    //creating matrix for input image
    Mat TheInputImage;
    Mat TheOutputImage;
    //receiving data from input sample, and saving to TheInputImage
    if (IS_OK(pISample->Lock(&l_pSrcBuffer)))
        {
        //convert to mat
        TheInputImage  = Mat(m_sInputFormat.nHeight,m_sInputFormat.nWidth,CV_8UC3,(tVoid*)l_pSrcBuffer,m_sInputFormat.nBytesPerLine);     
        pISample->Unlock(l_pSrcBuffer); 
///////////////start

    	if(!direct_flag)
	{
    	mytemplate = imread("/home/aadc/AADC/src/aadcUser/src/AADC_initpark/temp5.png",CV_LOAD_IMAGE_GRAYSCALE);
    	//Size size(128,96);
   	//resize(mytemplate,mytemplate,size);

    	//GaussianBlur( mytemplate,mytemplate, Size(7,7), 3.0 );
    	//cv::threshold(mytemplate, mytemplate, 0, 255, CV_THRESH_BINARY | THRESH_OTSU) ;  
    	LOG_INFO(adtf_util::cString::Format(" %d x %d ", mytemplate.rows, mytemplate.cols));
    	//threshold(mytemplate, mytemplate, 0, 255, CV_THRESH_BINARY | THRESH_OTSU) ;  

   	// cv::cvtColor(mytemplate, mytemplate, CV_BGR2GRAY);
    	//adaptiveThreshold( img_scene, img_scene, 255,ADAPTIVE_THRESH_GAUSSIAN_C,CV_THRESH_BINARY,13, 1 );
    	//Canny(mytemplate, mytemplate, 50, 150, 3);
    	//GaussianBlur( mytemplate, mytemplate, Size(7,7), 3.0 );
    
	//img = TheInputImage ;
    	
    	img = TheInputImage ;
    	//Canny(img, img, 75, 150, 3);
    	//GaussianBlur( img, img, Size(7,7), 3.0 );
    	img = img(Rect(350,220,290,190));
    
    	// Flip the frame horizontally and add blur
   	 //cv::flip( img, img, 1 );
    	cv::cvtColor(img, img, CV_RGB2GRAY);
    	GaussianBlur( img, img, Size(7,7), 3.0 );
    	cv::threshold(img, img, 0, 255, CV_THRESH_BINARY | THRESH_OTSU) ;
    	//img1 = img(Rect(435,95,100,40));
    	//imwrite("/home/aadc/AADC/src/aadcUser/src/imgprocex1/temp4.png",img1);  
    	//adaptiveThreshold( img, img, 255,ADAPTIVE_THRESH_GAUSSIAN_C,CV_THRESH_BINARY,13, 1 );
    	Mat element = getStructuringElement( MORPH_RECT,Size( 10,5),Point( 0, 0 ) );
    	//dilate( low1, dst_low, element );
    	dilate( img, img, element );
    	//adaptiveThreshold( img, img, 255,ADAPTIVE_THRESH_GAUSSIAN_C,CV_THRESH_BINARY,13, 1 );
    	//Canny(img, img, 50, 150, 3);
    	//GaussianBlur( img, img, Size(7,7), 3.0 );


  	matchTemplate( img, mytemplate, result, CV_TM_SQDIFF_NORMED );
  	normalize( result, result, 0, 1, NORM_MINMAX, -1, Mat() );
    
    	//Point match =  minmax( result ); 

         minMaxLoc( result, 0, 0, &minLoc, &maxLoc, Mat() );
         matchLoc = minLoc;
         match = matchLoc;
         cv::cvtColor(img, img, CV_GRAY2RGB);
         rectangle( img, match, Point( match.x + mytemplate.cols , match.y + mytemplate.rows ), CV_RGB(255, 56, 255), 1 );
	 rectangle( TheInputImage, match, Point( match.x + mytemplate.cols , match.y + mytemplate.rows ), CV_RGB(255, 56, 255), 1 );
	 TheOutputImage =img.clone();
	 templateFound_flag = tTrue;

     	}
	else
	{
		TheOutputImage =TheInputImage.clone();
		templateFound_flag = tFalse;	
		markercheckPass_flag = tFalse;
	}


     	if(imshowflag == tTrue)
     	{ 
       		//imshow("result",result);
       		//imshow("img1",img1);
       		imshow("mytemplate",mytemplate);
       		imshow( "image", img );
       		waitKey(10);
     	}
	generateop(_clock->GetStreamTime()); 

	TheOutputImage =TheInputImage.clone();
	//writeLights(timeStamp);
///////////////////end  
        pNewSample->Update(pISample->GetTime(), TheOutputImage.data, m_sOutputFormat.nSize, 0);
        m_oVideoOutputPin.Transmit(pNewSample);    
        }    

    RETURN_NOERROR;
}

tResult cParallelParkInit::UpdateInputImageFormat(const tBitmapFormat* pFormat)
{
    if (pFormat != NULL)
    {
        m_sInputFormat = (*pFormat);
        
        LOG_INFO(adtf_util::cString::Format("Marker Detection Filter: Input: Size %d x %d ; BPL %d ; Size %d , PixelFormat; %d", m_sInputFormat.nWidth,m_sInputFormat.nHeight,m_sInputFormat.nBytesPerLine, m_sInputFormat.nSize, m_sInputFormat.nPixelFormat));        
        
    }
    
    RETURN_NOERROR;
}

tResult cParallelParkInit::UpdateOutputImageFormat(const tBitmapFormat* pFormat)
{
    if (pFormat != NULL)
    {
        m_sOutputFormat = (*pFormat);    
        
        LOG_INFO(adtf_util::cString::Format("Marker Detection Filter: Output: Size %d x %d ; BPL %d ; Size %d, PixelFormat; %d", m_sOutputFormat.nWidth,m_sOutputFormat.nHeight,m_sOutputFormat.nBytesPerLine, m_sOutputFormat.nSize, m_sOutputFormat.nPixelFormat));
        
        m_oVideoOutputPin.SetFormat(&m_sOutputFormat, NULL);                
    }
    
    RETURN_NOERROR;
}
tResult cParallelParkInit::generateop(tUInt32 timeStamp)
{       





    	cObjectPtr<IMediaSample> pMediaSamplemarkerCheckPass;
    	cObjectPtr<IMediaSample> pMediaSampletemplateFoundPass;
    	cObjectPtr<IMediaSample> pMediaSampleBrakeLights;


	AllocMediaSample((tVoid**)&pMediaSamplemarkerCheckPass);
	AllocMediaSample((tVoid**)&pMediaSampletemplateFoundPass);
	AllocMediaSample((tVoid**)&pMediaSampleBrakeLights);



	cObjectPtr<IMediaSerializer>pSerializermarkerCheckPass;
	m_markercheckPass_flag->GetMediaSampleSerializer(&pSerializermarkerCheckPass);
	tInt nSizemarkerCheckPass = pSerializermarkerCheckPass->GetDeserializedSize();


	pMediaSamplemarkerCheckPass->AllocBuffer(nSizemarkerCheckPass);
	cObjectPtr<IMediaCoder> pCoderOutputmarkerCheckPass;

	m_markercheckPass_flag->WriteLock(pMediaSamplemarkerCheckPass, &pCoderOutputmarkerCheckPass);
	pCoderOutputmarkerCheckPass->Set("bValue", (tVoid*)&(markercheckPass_flag));
	pCoderOutputmarkerCheckPass->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
	m_markercheckPass_flag->Unlock(pCoderOutputmarkerCheckPass);
	pMediaSamplemarkerCheckPass->SetTime(_clock->GetStreamTime());
	m_omarkerCheckPass.Transmit(pMediaSamplemarkerCheckPass);




	cObjectPtr<IMediaSerializer>pSerializertemplateFoundPass;
	m_markercheckPass_flag->GetMediaSampleSerializer(&pSerializertemplateFoundPass);
	tInt nSizetemplateFoundPass = pSerializertemplateFoundPass->GetDeserializedSize();


	pMediaSampletemplateFoundPass->AllocBuffer(nSizetemplateFoundPass);
	cObjectPtr<IMediaCoder> pCoderOutputtemplateFoundPass;

	m_markercheckPass_flag->WriteLock(pMediaSampletemplateFoundPass, &pCoderOutputtemplateFoundPass);
	pCoderOutputtemplateFoundPass->Set("bValue", (tVoid*)&(templateFound_flag));
	pCoderOutputtemplateFoundPass->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
	m_markercheckPass_flag->Unlock(pCoderOutputtemplateFoundPass);
	pMediaSampletemplateFoundPass->SetTime(_clock->GetStreamTime());
	m_otemplateFoundPass.Transmit(pMediaSampletemplateFoundPass);

	


	RETURN_NOERROR;

}


