#ifndef _DEMO_START_PARALLEL_PARK_FILTER_CLASS_HEADER_
#define _DEMO_START_PARALLEL_PARK_FILTER_CLASS_HEADER_

#define OID_ADTF_DEMO_START_PARALLEL_PARK "adtf.example.startparallelpark"

class cParallelParkInit : public cFilter
{
        ADTF_FILTER(OID_ADTF_DEMO_START_PARALLEL_PARK, "Parallel Park Initialize", OBJCAT_DataFilter)  
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
    /*! input Pin for video */
    cVideoPin m_oVideoInputPin; 
    cInputPin m_signDetect;                
    /*! output Pin for video */
    cVideoPin m_oVideoOutputPin;                
    /*! output Pin for detected Sign as tInt */
    cOutputPin m_omarkerCheckPass;     
    cOutputPin m_otemplateFoundPass;       

    cObjectPtr<IMediaTypeDescription> m_markercheckPass_flag;
    cObjectPtr<IMediaTypeDescription> m_templateFound_flag;

    /*cObjectPtr<IMediaTypeDescription> m_pmaneuverleft_flag;
    cObjectPtr<IMediaTypeDescription> m_pmaneuverright_flag;*/
	cOutputPin    m_oXgoal;
	cOutputPin    m_oYgoal;

	cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalXgoal;
	cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalYgoal;
		

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
 	tFloat32 Xdesired;
	tFloat32 Ydesired;
	tBool indicatorLeft;
	tBool indicatorRight;
	tBool headLights;
    tBool m_bDebugModeEnabled;
    tBool enable_video_detection;
    tBool parkstart_flag;
    tBool m_bFirstFrame;
    tBool maneuverfinish_flag;
    tBool markercheckPass_flag;
    tBool templateFound_flag;
    tBool imshowflag;
    tBool direct_flag;
    tInt row1,row2,col1,col2;
    tUInt8      m_ui8Imagecount,harriscorner_count;
    tInt        m_nThresholdValue,m_cornerthreshValue,waitdelay;
	//tFloat32 f32RequiredArea ; // Area to be checked for marker area
	tUInt32 timeStamp;
	tInt16 i16ID;
	tFloat32 f32Area;
	tFloat32 f32RequiredArea;

	Point point1, point2 ,minLoc, maxLoc, matchLoc , match;; /* vertical points of the bounding box */
	Rect rect; /* bounding box */
	Mat img,roiImg,mytemplate, result,img1; /* roiImg - the part of the image in the bounding box */
	//tInt8 select_flag = 0;
	//tInt8 drag = 0;
                     
 
    tResult ProcessVideo(IMediaSample* pSample);
    tResult generateop(tUInt32 timeStamp);
    /*! function to set the m_sProcessFormat and the  m_sInputFormat variables
    @param pFormat the new format for the input and input pin
    */
    tResult UpdateInputImageFormat(const tBitmapFormat* pFormat);

    /*! function to set the m_output image format
    @param pFormat the new format for the output pin
    */
	tResult writeLights(tUInt32 timeStamp);
    tResult UpdateOutputImageFormat(const tBitmapFormat* pFormat);    
    tResult ReadProperties(const tChar* strPropertyName = NULL);
    //tVoid mouseHandler(tInt8 event, tInt8 x, tInt8 y, tInt8 flags, tVoid *param);
    //static tVoid mouseWrapper(tInt8 event, tInt8 x, tInt8 y, tInt8 flags, tVoid *param) ;


};
#endif //_MARKERDETECTFILTER_HEADER
