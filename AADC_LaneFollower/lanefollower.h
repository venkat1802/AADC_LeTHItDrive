#ifndef _LaneFollower_FILTER_HEADER_
#define _LaneFollower_FILTER_HEADER_

#define OID_ADTF_LaneFollower  "adtf.aadc.LaneFollower"

//! class for lane detection
/*!
This class was developed as a prototyp for a lane detection in the video stream. It can be used as a prototyp for further development.
*/
class cLaneFollower : public adtf::cFilter
{
	ADTF_DECLARE_FILTER_VERSION(OID_ADTF_LaneFollower, "AADC LaneFollower", adtf::OBJCAT_Tool, "AADC LaneFollower", 1, 0, 0, "Beta Version");
protected:
	//Eingang für RGB Bild
	cVideoPin		m_oVideoInputPin;		/**< the input pin for the video*/
	cInputPin       m_oEnable;
	cInputPin       m_oFreezeThink;
	cInputPin       m_oFreezeEB;
	cInputPin       m_oStop;
	cOutputPin      m_oSteer;
	cOutputPin      m_oAccelerate;
	cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalInputEnable;
	cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalInputFreezeEB;
	cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalInputFreezeThink;
	cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalInputStop;
	cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalOutputSteer;
	cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalOutputAccel;

public:
	tUInt8 blind_count;
	tFloat32 steeringAng;
	tBool enable;
	tBool enabled;
	tBool freezeEB;
	tBool freezeThink;
	tBool stopDriver;
    tBool freezedThink,enable_imshow;
	tFloat32 accel;
	tFloat32 steeringAngle;
	tFloat32 steeringAngle_previous;
	tFloat32 mean_theta_previous;
	tFloat32 KpsiRIGHT, KpsiLEFT, Ky;
	tFloat32 vMax, vMin;
    tBool writeZeroOutputs;
	tInt row1,row2,col1,col2;
	tInt cameraoffest,thresholdvalue,houghlinesvalue;

	cLaneFollower(const tChar*);
	virtual ~cLaneFollower();
	tResult Init(tInitStage eStage, __exception = NULL);
	tResult Shutdown(tInitStage eStage, ucom::IException** __exception_ptr = NULL);
	tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);

	tFloat32 evaluateSteeringAngle(IMediaSample* pSample);

	tResult UpdateImageFormat(const tBitmapFormat* pFormat);

	tResult PropertyChanged(const char* strProperty);

    tResult writeOutputs(tUInt32 timeStamp);



	cv::Mat Line;					/**< matrix for the line*/
	cv::Mat grey;					/**< matrix for the gray image*/
	cv::Mat greythresh;				/**< matrix for the gray threshold*/
	cv::Size cannysize;				/**< size for the canny detector*/
	cv::Mat linecanny;				/**< size for the canny lines*/

private:
	tBool firstFrame;				/**< flag for the first frame*/
	tUInt8 imagecount;				/**< counter for the images*/
	tBitmapFormat m_sInputFormat;	/**< bitmap format of the input image*/

};

#endif 

