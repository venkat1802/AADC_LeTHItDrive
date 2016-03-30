/**
Copyright (c) 
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: �This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.�
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS �AS IS� AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: spiesra $  $Date:: 2015-05-13 08:29:07#$ $Rev:: 35003   $
**********************************************************************/

#ifndef _MARKERSTRAIGHT_HEADER
#define _MARKERSTRAIGHT_HEADER
#define OID_ADTF_MARKERSTRAIGHT "adtf.aadc.aadc_markerstraight"

class cMarkerStraightFilter : public cFilter
{
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_MARKERSTRAIGHT, "AADC Marker Straight Filter", adtf::OBJCAT_Tool, "AADC Marker Straight Filter", 1, 0, 1, "BFFT GmbH");
public:

    cMarkerStraightFilter(const tChar* __info);
    virtual ~cMarkerStraightFilter();
    tResult Init(tInitStage eStage, __exception);    
    tResult OnPinEvent(adtf::IPin* pSource, tInt nEventCode,tInt nParam1,tInt nParam2,adtf::IMediaSample* pMediaSample);
    tResult Shutdown(tInitStage eStage, __exception = NULL);

    tResult PropertyChanged(const char* strProperty);

protected:
    /*! input Pin for video */
    cVideoPin m_oPinInputVideo;                 
    /*! output Pin for video */
    cVideoPin m_oPinOutputVideo;                
    /*! output Pin for detected Sign as tInt */
    cOutputPin m_oPinRoadSign;            
    /*! output Pin for detected Sign as tInt with extended information */
    cOutputPin m_oPinRoadSignExt;
    cOutputPin m_headlight;

    cInputPin maneuver_finish;
    cInputPin maneuverid_Jury;
    cOutputPin maneuver_id;
    cObjectPtr<IMediaTypeDescription> med_typ_m_headlight;
    cObjectPtr<IMediaTypeDescription> mediatype_maneuver_finish;
    cObjectPtr<IMediaTypeDescription> mediatype_maneuver_id;
    cObjectPtr<IMediaTypeDescription> mediatype_maneuverid_Jury;
    /*!  */    
    UCOM_DECLARE_TIMING_SPOT(m_oProcessStart)
    /*!  */        
    UCOM_DECLARE_TIMING_SPOT(m_oProcessEnd)

private:
    tResult ReadProperties(const tChar* strPropertyName = NULL);
    tInt FrameCount;
    tBool var_finish_flag;//manuever_enable_flag;
    tFloat32 f32RequiredArea;
    tFloat32 ManeuverJuryID;
    tInt row1,row2,col1,col2;
    tInt row1_ver,row2_ver,col1_ver,col2_ver;
    tInt row1_l,row2_l,col1_l,col2_l,count_lines_marker;
    /*! bitmapformat of input image */    
    tBitmapFormat      m_sInputFormat;                        

    /*! bitmapformat of output image */    
    tBitmapFormat      m_sOutputFormat;
    /*! indicates wheter information is printed to the console or not */
    tBool m_bDebugModeEnabled;

    /*! indicates wheter the camara parameters are loaded or not */
    tBool m_bCamaraParamsLoaded;

    /*! indicates what is transmitted over the output pin*/
    tInt m_iOutputMode;

    /*! Descriptor */
    cObjectPtr<IMediaTypeDescription> m_pDescriptionRoadSign;        
    /*! the id for the i16Identifier of the media description for output pin */
    tBufferID m_szIDRoadSignI16Identifier; 
    /*! the id for the f32Imagesize of the media description for output pin */
    tBufferID m_szIDRoadSignF32Imagesize; 
    /*! indicates if bufferIDs were set */
    tBool m_bIDsRoadSignSet;

    /*! Descriptor */
    cObjectPtr<IMediaTypeDescription> m_pDescriptionRoadSignExt;        
    /*! the id for the i16Identifier of the media description for output pin */
    tBufferID m_szIDRoadSignExtI16Identifier; 
    /*! the id for the f32Imagesize of the media description for output pin */
    tBufferID m_szIDRoadSignExtF32Imagesize; 
    /*! the id for the af32TVec of the media description for output pin */
    tBufferID m_szIDRoadSignExtAf32TVec; 
    /*! the id for the af32RVec of the media description for output pin */
    tBufferID m_szIDRoadSignExtAf32RVec; 
    /*! indicates if bufferIDs were set */
    tBool m_bIDsRoadSignExtSet;

    //void writeoutput_laneAcc(tUInt32 timeStamp);
    /*! function process the video data
    @param pSample the new media sample to be processed


    */    
    tResult ProcessVideo(IMediaSample* pSample);

    /*! function to set the m_sProcessFormat and the  m_sInputFormat variables
    @param pFormat the new format for the input and input pin
    */
    tResult UpdateInputImageFormat(const tBitmapFormat* pFormat);

    /*! function to set the m_output image format
    @param pFormat the new format for the output pin
    */
    tResult UpdateOutputImageFormat(const tBitmapFormat* pFormat);    

    /*! function to transmit a detected road sign
    @param i16ID ID of the sign
    @param f32MarkerSize size of the markers sides in meters
    @param timeOfFrame the timestamp of the frame where the sign was detected
    */
    tResult sendRoadSignStruct(const tInt16 &i16ID, const tFloat32 &f32MarkerSize, const tTimeStamp &timeOfFrame);

    /*! function to transmit a detected road sign with extedend info
    @param i16ID ID of the sign
    @param f32MarkerSize size of the markers sides in meters
    @param timeOfFrame the timestamp of the frame where the sign was detected
    @param Tvec the translation vector
    @param Rvec the rotation vector
    */
    tResult sendRoadSignStructExt(const tInt16 &i16ID, const tFloat32 &f32MarkerSize, const tTimeStamp &timeOfFrame, const Mat &Tvec, const Mat &Rvec);

    /*! the aruco elements: */
    /*! the aruco detector for the markers*/
    MarkerDetector m_MDetector;
    /*! the aruco markers */
    vector<Marker> m_TheMarkers_org;
    vector<Marker> m_TheMarkers;    
    /*! size of the markers*/ 
    tFloat32 m_f32MarkerSize,var_maneuver_id;
    /*! the intrinsic parameter of the camera*/
    CameraParameters m_TheCameraParameters;
    /*! the dictionary for the aruco lib*/ 
    Dictionary m_Dictionary;

    cv::Mat m_Intrinsics; 
    cv::Mat m_Distorsion;  

    tBool imshowflag,imshowflag_ver;
    tInt  m_nThresholdValue;
};
#endif //_MARKERDETECTFILTER_HEADER
