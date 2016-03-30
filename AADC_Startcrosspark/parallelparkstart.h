#ifndef _DEMO_START_PARALLEL_PARK_FILTER_CLASS_HEADER_
#define _DEMO_START_PARALLEL_PARK_FILTER_CLASS_HEADER_

#define OID_ADTF_DEMO_START_PARALLEL_PARK "adtf.example.startparallelpark"

class cParallelParkInit : public cFilter
{
        ADTF_FILTER(OID_ADTF_DEMO_START_PARALLEL_PARK, "Parallel Parking", OBJCAT_DataFilter)  
public:

    cParallelParkInit(const tChar* __info);
    virtual ~cParallelParkInit();


    tResult Init(tInitStage eStage, __exception);    
    tResult OnPinEvent(adtf::IPin* pSource, tInt nEventCode,tInt nParam1,tInt nParam2,adtf::IMediaSample* pMediaSample);
    tResult Shutdown(tInitStage eStage, __exception = NULL);
    tResult PropertyChanged(const char* strProperty);
    tResult Start(__exception);
    tResult Stop(__exception);




protected:




    // video input and output pins
    cVideoPin     m_oVideoInputPin;  //video input    
    cVideoPin     m_oVideoOutputPin;   //video output    
    
    
    //actuator and control inputs
	cInputPin     m_oUSRight;	     //free spot check  
	cInputPin     m_oUSRear;
	cInputPin     m_oLFaccel;        //input accel
	cInputPin	  m_oLFsteer;
	cInputPin	  m_oDistance;
	//cInputPin	  m_oYaw;   
	
	// extra inputs	
	//cInputPin     m_ojuryinput;      //input command  //currently my start flag
	cInputPin     m_osignDetect;      //detected sign   
	
	
	//control outputs  
	cOutputPin    m_oManeuverFinish;  //full maneuver
	
	//actuator outputs
	cOutputPin    m_oBrakeLights;
	cOutputPin    m_oIndicatorLeft;
	cOutputPin    m_oIndicatorRight;
	cOutputPin    m_oHeadLights;
	
	cOutputPin    m_oSteerOut;	
	cOutputPin    m_oAccelOut;
	cOutputPin    m_oSignalID;
	//cOutputPin    m_oYval;

	tUInt32 timeStamp;

    
    //control output description
    cObjectPtr<IMediaTypeDescription>    m_pCoderDescpSignalManevuerFinish;
	//control input description
	cObjectPtr<IMediaTypeDescription>    m_pCoderDescSignalUSRight;	
	cObjectPtr<IMediaTypeDescription>    m_pCoderDescSignalUSRear;	
	cObjectPtr<IMediaTypeDescription>    m_pCoderDescSignalLFaccel;
	cObjectPtr<IMediaTypeDescription>    m_pCoderDescSignalLFsteer;
	cObjectPtr<IMediaTypeDescription>    m_pCoderDescSignalDistance;
	//cObjectPtr<IMediaTypeDescription>    m_pCoderDescSignalYaw;


	//actuator output description
	cObjectPtr<IMediaTypeDescription>    m_pCoderDescpSignalAccelOut;
	cObjectPtr<IMediaTypeDescription>    m_pCoderDescpSignalSignalID;
	//cObjectPtr<IMediaTypeDescription>    m_pCoderDescpSignalYval;
	
	//status output description
	cObjectPtr<IMediaTypeDescription> 	 m_pCoderDescpSignalBrakeLights;
	cObjectPtr<IMediaTypeDescription> 	 m_pCoderDescpSignalHeadLights;
	cObjectPtr<IMediaTypeDescription> 	 m_pCoderDescpSignalIndicatorLeft;
	cObjectPtr<IMediaTypeDescription> 	 m_pCoderDescpSignalIndicatorRight;
	cObjectPtr<IMediaTypeDescription>	 m_pCoderDescpSignalSteerOut;



	//roadsign description
    cObjectPtr<IMediaTypeDescription>    m_pCoderDescSignaljuryinput;
    /*! Descriptor roadsign */
    cObjectPtr<IMediaTypeDescription> m_pDescriptionRoadSign;        
    /*! the id for the i16Identifier of the media description for output pin */
    tBufferID m_szIDRoadSignI16Identifier; 
    /*! the id for the f32Imagesize of the media description for output pin */
    tBufferID m_szIDRoadSignF32Imagesize; 
    /*! indicates if bufferIDs were set */
    tBool m_bIDsRoadSignSet;


private:
    /*! bitmapformat of input image */    
    tBitmapFormat      m_sInputFormat;                        

    /*! bitmapformat of output image */    
    tBitmapFormat      m_sOutputFormat; 
    
    
// input pin variables
	tFloat32 USRight;
	tFloat32 accelin;
	tFloat32 Distnce;
	tFloat32 Steerin;
	tFloat32 parkaccel;
	tFloat32 parkaccelrev;
	tFloat32 stg3str;
	tFloat32 yaw;
	//roadsign vars
	tInt16 i16ID;
	tFloat32 f32Area;
	tFloat32 f32RequiredArea;
	//jury flag
	//tBool juryinputflag;      // use for start atm

// output pin variables
	tFloat32 SignalID;
	//tFloat32 Yval;
	tFloat32 Steerout;
	tFloat32 accelout;	
	tBool brakeLights; 
	tBool indicatorLeft;
	tBool indicatorRight;
	tBool headLights;	
	tBool HazardLight;
	//flags for debugging
	tInt Temp_count; //temp count
	tFloat32 maneuverfinish_flag;   
    tBool markercheckPassFlag;
    tBool templateFoundFlag;
	


//control flags
	tInt32 currentStage;
	tBool stageStartflag;
	tBool stageFinishflag;	

	

    tBool m_bDebugModeEnabled;
    tBool enable_video_detection;
    
    tBool maneuverstart_flag;  // from jury
    tBool m_bFirstFrame;

    tBool imshowflag;
    tBool direct_flag;
    
    tInt row1,row2,col1,col2;
    //tUInt8      m_ui8Imagecount,harriscorner_count;
    //tInt        m_nThresholdValue,m_cornerthreshValue,waitdelay;
	//tFloat32 f32RequiredArea ; // Area to be checked for marker area



	Point point1, point2 ,minLoc, maxLoc, matchLoc , match,match_old; /* vertical points of the bounding box */
	Rect rect; /* bounding box */
	Mat img,roiImg,mytemplate, result,img1; /* roiImg - the part of the image in the bounding box */
	//tInt8 select_flag = 0;
	//tInt8 drag = 0;
	tBool frame_noise;
	tBool spot;
	tInt32 count1;
	tInt32 total_count;
	tInt32 total_count_old ;
    tBool found1;
	tBool foundother;
	tFloat32 distance_begin;
	tFloat32 distance_desired1;
	tFloat32 distance_desired2;
	tFloat32 distance_desired3;
	tFloat32 distance_desired4;
	tFloat32 distance_desired5;
	tFloat32 distance_desired6;
	tFloat32 distance_desired7;


	tFloat32 distance_add1;
	tFloat32 distance_add2;
	tFloat32 distance_add3;
	tFloat32 distance_add4;
	tFloat32 distance_add5;
	tFloat32 distance_add6;
	tFloat32 distance_add7;                    
 
    tResult ProcessVideo(IMediaSample* pSample);
    tResult DecisionMaking(tUInt32 timeStamp);
    tResult TakeAction(tUInt32 timeStamp);
    tResult UpdateInputImageFormat(const tBitmapFormat* pFormat);
    tResult UpdateOutputImageFormat(const tBitmapFormat* pFormat);    
    tResult ReadProperties(const tChar* strPropertyName = NULL);
    tVoid thinningIteration(cv::Mat& img, tInt iter);
    tVoid thinning(const cv::Mat& src, cv::Mat& dst);

    
    
    //transmit flags //removed
	//tResult writeFlags(tUInt32 timeStamp);
	
	//transmit acutator outputs
	tResult writeOutputs(tUInt32 timeStamp);
	
	//writeLights
	tResult writeLights(tUInt32 timeStamp);

    
    
    //tVoid mouseHandler(tInt8 event, tInt8 x, tInt8 y, tInt8 flags, tVoid *param);
    //static tVoid mouseWrapper(tInt8 event, tInt8 x, tInt8 y, tInt8 flags, tVoid *param) ;


};
#endif //_MARKERDETECTFILTER_HEADER
