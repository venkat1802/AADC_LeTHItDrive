/**
Copyright (c) 
Audi Autonomous Driving Cup. All rights reserved.
 
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 
1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: â€œThis product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.â€�
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 
THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS â€œAS ISâ€� AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: spiesra $  $Date:: 2015-05-13 08:29:07#$ $Rev:: 35003   $
**********************************************************************/

#include "stdafx.h"
#include "aadc_roadSign_enums.h"
#include "cMarkerStraight.h"

#define MD_RANGE_ROWS_R1 "MarkerDetection::ROW1"
#define MD_RANGE_ROWS_R2 "MarkerDetection::ROW2"
#define MD_RANGE_COLS_C1 "MarkerDetection::COL1"
#define MD_RANGE_COLS_C2 "MarkerDetection::COL2"
#define MD_RANGE_ROWS_R1_L "LineDetection::ROW1"
#define MD_RANGE_ROWS_R2_L "LineDetection::ROW2"
#define MD_RANGE_COLS_C1_L "LineDetection::COL1"
#define MD_RANGE_COLS_C2_L "LineDetection::COL2"

#define MD_RANGE_ROWS_R1_VER "LineDetection_ver::ROW1"
#define MD_RANGE_ROWS_R2_VER "LineDetection_ver::ROW2"
#define MD_RANGE_COLS_C1_VER "LineDetection_ver::COL1"
#define MD_RANGE_COLS_C2_VER "LineDetection_ver::COL2"


#define MD_COUNT "MarkerDetection::Count"

#define MD_REQUIRED_MARKER_SIZE "MarkerDetection::Required marker size"
#define MD_ENABLE_IMSHOW  "MarkerDetection::Enableimshow"
#define MD_ENABLE_IMSHOW_VER  "MarkerDetection::Enableimshow_VER"
#define MD_GREY_TRESHOLD "MarkerDetection::GreyThresholdValue"

// to check the input signal ids
#define START_PULLOVER_LEFT (tFloat32)1.0
#define START_PULLOVER_RIGHT (tFloat32)2.0
#define START_PULLOVER_PARALLEL (tFloat32)3.0
#define PULLOVER_STATUS_DEFAULT (tFloat32)4.0

#define MANEUVER_INTERSECTION_RIGHT      (tFloat32)5.0
#define MANEUVER_INTERSECTION_LEFT       (tFloat32)6.0
#define MANEUVER_INTERSECTION_STRAIGHT   (tFloat32)7.0
#define MANEUVER_INTERSECTION_DEFAULT   (tFloat32)10.0

#define MANEUVER_PARALLEL_PARKING        (tFloat32)14.0
#define MANEUVER_CROSS_PARKING           (tFloat32)15.0
#define MANEUVER_PARKING_DEFAULT         (tFloat32)16.0

int global_i = 0;
tFloat32 TestArray[9] = {MANEUVER_INTERSECTION_STRAIGHT,MANEUVER_INTERSECTION_STRAIGHT,MANEUVER_INTERSECTION_STRAIGHT,MANEUVER_INTERSECTION_STRAIGHT,MANEUVER_INTERSECTION_STRAIGHT,MANEUVER_INTERSECTION_STRAIGHT,MANEUVER_INTERSECTION_STRAIGHT,
MANEUVER_INTERSECTION_STRAIGHT,MANEUVER_INTERSECTION_STRAIGHT};

ADTF_FILTER_PLUGIN("Marker Straight Filter Plugin", OID_ADTF_MARKERSTRAIGHT, cMarkerStraightFilter)

tResult cMarkerStraightFilter::PropertyChanged(const char* strProperty)
{
    ReadProperties(strProperty);
    RETURN_NOERROR;
}

tResult cMarkerStraightFilter::ReadProperties(const tChar* strPropertyName)
{
    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, MD_COUNT))
    {
    	count_lines_marker = GetPropertyInt(MD_COUNT);
    }
    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, MD_REQUIRED_MARKER_SIZE))
    {
    	f32RequiredArea = GetPropertyFloat(MD_REQUIRED_MARKER_SIZE);
    }
    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, MD_GREY_TRESHOLD))
    {
        m_nThresholdValue = GetPropertyInt(MD_GREY_TRESHOLD);
    }
    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, MD_RANGE_ROWS_R1))
    {
        row1 = GetPropertyInt(MD_RANGE_ROWS_R1);
    }
    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, MD_RANGE_ROWS_R2))
    {
    	row2 = GetPropertyInt(MD_RANGE_ROWS_R2);
    }
    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, MD_RANGE_COLS_C1))
    {
        col1 = GetPropertyInt(MD_RANGE_COLS_C1);
    }
    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, MD_RANGE_COLS_C2))
    {
    	col2 = GetPropertyInt(MD_RANGE_COLS_C2);
    }
    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, MD_RANGE_ROWS_R1_L))
    {
        row1_l = GetPropertyInt(MD_RANGE_ROWS_R1_L);
    }
    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, MD_RANGE_ROWS_R2_L))
    {
    	row2_l = GetPropertyInt(MD_RANGE_ROWS_R2_L);
    }
    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, MD_RANGE_COLS_C1_L))
    {
        col1_l = GetPropertyInt(MD_RANGE_COLS_C1_L);
    }
    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, MD_RANGE_COLS_C2_L))
    {
    	col2_l = GetPropertyInt(MD_RANGE_COLS_C2_L);
    }
    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, MD_RANGE_ROWS_R1_VER))
    {
        row1_ver = GetPropertyInt(MD_RANGE_ROWS_R1_VER);
    }
    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, MD_RANGE_ROWS_R2_VER))
    {
    	row2_ver = GetPropertyInt(MD_RANGE_ROWS_R2_VER);
    }
    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, MD_RANGE_COLS_C1_VER))
    {
        col1_ver = GetPropertyInt(MD_RANGE_COLS_C1_VER);
    }
    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, MD_RANGE_COLS_C2_VER))
    {
    	col2_ver = GetPropertyInt(MD_RANGE_COLS_C2_VER);
    }
	RETURN_NOERROR;
}

cMarkerStraightFilter::cMarkerStraightFilter(const tChar* __info):cFilter(__info)
{
    UCOM_REGISTER_TIMING_SPOT(cString(OIGetInstanceName()) + "::Process::Start", m_oProcessStart);

    global_i = 0;
    var_finish_flag = tTrue;
    FrameCount = 0;
    ManeuverJuryID = MANEUVER_INTERSECTION_DEFAULT;
    var_maneuver_id = TestArray[global_i];//(tFloat32)MANEUVER_INTERSECTION_RIGHT; //TestArray[global_i]; //global_i = 0;
    SetPropertyBool(MD_ENABLE_IMSHOW,tFalse);
    SetPropertyBool(MD_ENABLE_IMSHOW NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(MD_ENABLE_IMSHOW NSSUBPROP_DESCRIPTION, "If true imshow will be enabled");

    SetPropertyBool(MD_ENABLE_IMSHOW_VER,tFalse);
    SetPropertyBool(MD_ENABLE_IMSHOW_VER NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(MD_ENABLE_IMSHOW_VER NSSUBPROP_DESCRIPTION, "If true imshow will be enabled for vertical lines");

    SetPropertyBool("Debug Output to Console", tFalse);    
    SetPropertyStr("Debug Output to Console" NSSUBPROP_DESCRIPTION, "If enabled additional debug information is printed to the console (Warning: decreases performance)."); 

    //RETURN_NOERROR;
    SetPropertyStr("Dictionary File For Markers",""); 
    SetPropertyBool("Dictionary File For Markers" NSSUBPROP_FILENAME, tTrue); 
    SetPropertyStr("Dictionary File For Markers" NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "YML Files (*.yml)"); 
    SetPropertyStr("Dictionary File For Markers" NSSUBPROP_DESCRIPTION, "Here you have to set the dicitionary file which holds the marker ids and their content"); 

    SetPropertyStr("Calibration File for used Camera",""); 
    SetPropertyBool("Calibration File for used Camera" NSSUBPROP_FILENAME, tTrue); 
    SetPropertyStr("Calibration File for used Camera" NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "YML Files (*.yml)"); 
    SetPropertyStr("Calibration File for used Camera" NSSUBPROP_DESCRIPTION, "Here you have to set the file with calibration paraemters of the used camera"); 
    
    SetPropertyInt("Video Output Pin",1);
    SetPropertyStr("Video Output Pin" NSSUBPROP_VALUELIST, "1@None|2@Detected Signs|");
    SetPropertyStr("Video Output Pin" NSSUBPROP_DESCRIPTION, "If enabled the video stream with the highlighted markers is transmitted at the output pin (Warning: decreases performance)."); 

    SetPropertyFloat("Size of Markers",0.117f);
    SetPropertyStr("Size of Markers" NSSUBPROP_DESCRIPTION, "Size (length of one side) of markers in m"); 

    SetPropertyFloat(MD_REQUIRED_MARKER_SIZE, 900.0);
    SetPropertyBool(MD_REQUIRED_MARKER_SIZE NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(MD_REQUIRED_MARKER_SIZE NSSUBPROP_DESCRIPTION, "MarkerDetection::The marker size above which manuevers processing starts");

    SetPropertyInt(MD_COUNT, 2);
    SetPropertyBool(MD_COUNT NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(MD_COUNT NSSUBPROP_DESCRIPTION, "MarkerDetection::Minimum number of lines to be detected for markers to be considered");

    SetPropertyInt(MD_RANGE_ROWS_R1, 0);
    SetPropertyBool(MD_RANGE_ROWS_R1 NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(MD_RANGE_ROWS_R1 NSSUBPROP_DESCRIPTION, "MarkerDetection::The first row R1 to be taken (R1,) (R2,)");

    SetPropertyInt(MD_RANGE_ROWS_R2, 250);
    SetPropertyBool(MD_RANGE_ROWS_R2 NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(MD_RANGE_ROWS_R2 NSSUBPROP_DESCRIPTION,"MarkerDetection::The second row R2 to be taken (R1,) (R2,)");

    SetPropertyInt(MD_RANGE_COLS_C1, 0);
    SetPropertyBool(MD_RANGE_COLS_C1 NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(MD_RANGE_COLS_C1 NSSUBPROP_DESCRIPTION, "MarkerDetection::The first col C1 to be taken (,C1) (,C2)");

    SetPropertyInt(MD_RANGE_COLS_C2, 250);
    SetPropertyBool(MD_RANGE_COLS_C2 NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(MD_RANGE_COLS_C2 NSSUBPROP_DESCRIPTION, "MarkerDetection::The second col C2 to be taken (,C1) (,C2)");

    SetPropertyInt(MD_RANGE_ROWS_R1_L, 20);
    SetPropertyBool(MD_RANGE_ROWS_R1_L NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(MD_RANGE_ROWS_R1_L NSSUBPROP_DESCRIPTION, "LineDetection::The first row R1 to be taken (R1,) (R2,)");

    SetPropertyInt(MD_RANGE_ROWS_R2_L, 50);
    SetPropertyBool(MD_RANGE_ROWS_R2_L NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(MD_RANGE_ROWS_R2_L NSSUBPROP_DESCRIPTION,"LineDetection::The second row R2 to be taken (R1,) (R2,)");

    SetPropertyInt(MD_RANGE_COLS_C1_L, 90);
    SetPropertyBool(MD_RANGE_COLS_C1_L NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(MD_RANGE_COLS_C1_L NSSUBPROP_DESCRIPTION, "LineDetection::The first col C1 to be taken (,C1) (,C2)");

    SetPropertyInt(MD_RANGE_COLS_C2_L, 20);
    SetPropertyBool(MD_RANGE_COLS_C2_L NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(MD_RANGE_COLS_C2_L NSSUBPROP_DESCRIPTION, "LineDetection::The second col C2 to be taken (,C1) (,C2)");

    SetPropertyInt(MD_RANGE_ROWS_R1_VER, 30);
    SetPropertyBool(MD_RANGE_ROWS_R1_VER NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(MD_RANGE_ROWS_R1_VER NSSUBPROP_DESCRIPTION, "LineDetection_VER::Theq first row R1 to be taken (R1,) (R2,)");

    SetPropertyInt(MD_RANGE_ROWS_R2_VER, 60);
    SetPropertyBool(MD_RANGE_ROWS_R2_VER NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(MD_RANGE_ROWS_R2_VER NSSUBPROP_DESCRIPTION,"LineDetection_VER::Theq second row R2 to be taken (R1,) (R2,)");

    SetPropertyInt(MD_RANGE_COLS_C1_VER, 30);
    SetPropertyBool(MD_RANGE_COLS_C1_VER NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(MD_RANGE_COLS_C1_VER NSSUBPROP_DESCRIPTION, "LineDetection_VER::Theq first col C1 to be taken (,C1) (,C2)");

    SetPropertyInt(MD_RANGE_COLS_C2_VER, 60);
    SetPropertyBool(MD_RANGE_COLS_C2_VER NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(MD_RANGE_COLS_C2_VER NSSUBPROP_DESCRIPTION, "LineDetection_VER::Theq second col C2 to be taken (,C1) (,C2)");

    SetPropertyInt(MD_GREY_TRESHOLD, 100);
    SetPropertyBool(MD_GREY_TRESHOLD NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(MD_GREY_TRESHOLD NSSUBPROP_DESCRIPTION, "The threshold value for canny to detect lines.");

    UCOM_REGISTER_TIMING_SPOT(cString(OIGetInstanceName()) + "::Process::End", m_oProcessEnd);    
}

cMarkerStraightFilter::~cMarkerStraightFilter()
{
}

tResult cMarkerStraightFilter::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));
    if (eStage == StageFirst)
        {
        //create the video rgb input pin
        RETURN_IF_FAILED(m_oPinInputVideo.Create("Video_RGB_inputS",IPin::PD_Input, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oPinInputVideo));

        //create the video rgb output pin
        RETURN_IF_FAILED(m_oPinOutputVideo.Create("Video_RGB_outputS", IPin::PD_Output, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oPinOutputVideo));

        // create the description manager
        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**)&pDescManager,__exception_ptr));

		tChar const * strDescSignalmaneuver_idJury = pDescManager->GetMediaDescription("tSignalValue");
		RETURN_IF_POINTER_NULL(strDescSignalmaneuver_idJury);
		cObjectPtr<IMediaType> pTypeSignalmaneuver_idjury = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalmaneuver_idJury, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(pTypeSignalmaneuver_idjury->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&mediatype_maneuverid_Jury));
		RETURN_IF_FAILED(maneuverid_Jury.Create("maneuverid_JuryS", pTypeSignalmaneuver_idjury, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&maneuverid_Jury));

		tChar const * strDescSignalmaneuver_id = pDescManager->GetMediaDescription("tSignalValue");
		RETURN_IF_POINTER_NULL(strDescSignalmaneuver_id);
		cObjectPtr<IMediaType> pTypeSignalmaneuver_id = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalmaneuver_id, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(pTypeSignalmaneuver_id->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&mediatype_maneuver_id));
		RETURN_IF_FAILED(maneuver_id.Create("Maneuver_IDS", pTypeSignalmaneuver_id, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&maneuver_id));

		tChar const * strDescSignalmaneuver_finish = pDescManager->GetMediaDescription("tBoolSignalValue");
		RETURN_IF_POINTER_NULL(strDescSignalmaneuver_finish);
		cObjectPtr<IMediaType> pTypeSignalmaneuver_finish = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalmaneuver_finish, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(pTypeSignalmaneuver_finish->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&mediatype_maneuver_finish));
		RETURN_IF_FAILED(maneuver_finish.Create("Maneuver_int_FinishS", pTypeSignalmaneuver_finish, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&maneuver_finish));

		tChar const * med_des_m_headlight = pDescManager->GetMediaDescription("tBoolSignalValue");
		RETURN_IF_POINTER_NULL(med_des_m_headlight);
		cObjectPtr<IMediaType> med_typ_m_headlight_loc	= new cMediaType(0, 0, 0, "tBoolSignalValue", med_des_m_headlight, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(med_typ_m_headlight_loc->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&med_typ_m_headlight));
		RETURN_IF_FAILED(m_headlight.Create("Finish_FlagS", med_typ_m_headlight_loc, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_headlight));

        // create the description for the road sign pin
        tChar const * strDesc = pDescManager->GetMediaDescription("tRoadSign");   
        RETURN_IF_POINTER_NULL(strDesc);    
        cObjectPtr<IMediaType> pType = new cMediaType(0, 0, 0, "tRoadSign", strDesc, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        
        // create the road sign OutputPin
        RETURN_IF_FAILED(m_oPinRoadSign.Create("RoadSignS", pType, this));
        RETURN_IF_FAILED(RegisterPin(&m_oPinRoadSign));
        // set the description for the road sign pin
        RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionRoadSign));
                
        // create the description for the road sign pin
        tChar const * strDescExt = pDescManager->GetMediaDescription("tRoadSignExt");   
        RETURN_IF_POINTER_NULL(strDescExt);    
        cObjectPtr<IMediaType> pTypeExt = new cMediaType(0, 0, 0, "tRoadSignExt", strDescExt, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        
        // create the extended road sign OutputPin
        RETURN_IF_FAILED(m_oPinRoadSignExt.Create("RoadSign_extS", pTypeExt, this));
        RETURN_IF_FAILED(RegisterPin(&m_oPinRoadSignExt));
        // set the description for the extended road sign pin
        RETURN_IF_FAILED(pTypeExt->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionRoadSignExt));
        }
    else if (eStage == StageNormal)
        {
            // get the propeerties
    		ReadProperties(NULL);
            m_iOutputMode = GetPropertyInt("Video Output Pin");
            m_f32MarkerSize = static_cast<tFloat32>(GetPropertyFloat("Size of Markers"));
            m_bDebugModeEnabled = GetPropertyBool("Debug Output to Console");
            imshowflag = GetPropertyBool(MD_ENABLE_IMSHOW);
            imshowflag_ver = GetPropertyBool(MD_ENABLE_IMSHOW_VER);
            m_bCamaraParamsLoaded = tFalse;
            //Get path of marker configuration file
            cFilename fileConfig = GetPropertyStr("Dictionary File For Markers");
            //create absolute path for marker configuration file
            ADTF_GET_CONFIG_FILENAME(fileConfig);
            fileConfig = fileConfig.CreateAbsolutePath(".");
            
            //check if marker configuration file exits
            if (fileConfig.IsEmpty() || !(cFileSystem::Exists(fileConfig)))
                {
                LOG_ERROR("Dictionary File for Markers not found");
                RETURN_ERROR(ERR_INVALID_FILE);
                }   
            else
                {
                //try to read the marker configuration file
                if(m_Dictionary.fromFile(string(fileConfig))==false)
                    {
                    RETURN_ERROR(ERR_INVALID_FILE);
                    LOG_ERROR("Dictionary File for Markers could not be read");
                    }
                if(m_Dictionary.size()==0)
                    {
                    RETURN_ERROR(ERR_INVALID_FILE);
                    LOG_ERROR("Dictionary File does not contain valid markers or could not be read sucessfully");
                    }
                //set marker configuration file to highlyreliable markers class
                if (!(HighlyReliableMarkers::loadDictionary(m_Dictionary)==tTrue))
                    {
                    //LOG_ERROR("Dictionary File could not be read for highly reliable markers"); 
                    }
                } 
    
            //Get path of calibration file with camera paramters
            cFilename fileCalibration = GetPropertyStr("Calibration File for used Camera"); ;
        
            //Get path of calibration file with camera paramters
            ADTF_GET_CONFIG_FILENAME(fileCalibration);
            fileCalibration = fileCalibration.CreateAbsolutePath(".");
            //check if calibration file with camera paramters exits
            if (fileCalibration.IsEmpty() || !(cFileSystem::Exists(fileCalibration)))
                {
                LOG_ERROR("Calibration File for camera not found");
                }   
            else
                {
                // read the calibration file with camera paramters exits and save to member variable
                m_TheCameraParameters.readFromXMLFile(fileCalibration.GetPtr());
                cv::FileStorage camera_data (fileCalibration.GetPtr(),cv::FileStorage::READ);
                camera_data ["camera_matrix"] >> m_Intrinsics; 
                camera_data ["distortion_coefficients"] >> m_Distorsion;    
                m_bCamaraParamsLoaded = tTrue;
                }   
        }
    else if (eStage == StageGraphReady)
        {
        // get the image format of the input video pin
        cObjectPtr<IMediaType> pType;
        RETURN_IF_FAILED(m_oPinInputVideo.GetMediaType(&pType));

        cObjectPtr<IMediaTypeVideo> pTypeVideo;
        RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));
    
        // set the image format of the input video pin
        UpdateInputImageFormat(pTypeVideo->GetFormat());
        //TODO: why was the following line inserted?
        //m_oPinInputVideo.SetFormat(&m_sInputFormat, NULL);  
    
        // set the image format of the output video pin
        UpdateOutputImageFormat(pTypeVideo->GetFormat());  
    
        // set paramenter for the marker detector class
        // same parameters as in aruco sample aruco_hrm_test.cpp
        // effects of parameters has to be tested
        m_MDetector.setMakerDetectorFunction(aruco::HighlyReliableMarkers::detect);
        m_MDetector.setThresholdParams( 21, 7);
        m_MDetector.setCornerRefinementMethod(aruco::MarkerDetector::LINES);
        m_MDetector.setWarpSize((m_Dictionary[0].n()+2)*8);
        m_MDetector.setMinMaxSize(0.005f, 0.5f);

        // IDs were not set yet
        m_bIDsRoadSignExtSet = tFalse;
        m_bIDsRoadSignSet = tFalse;
        }
    RETURN_NOERROR;
}

tResult cMarkerStraightFilter::Shutdown(tInitStage eStage, __exception)
{
    return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult cMarkerStraightFilter::OnPinEvent(IPin* pSource,
                                            tInt nEventCode,
                                            tInt nParam1,
                                            tInt nParam2,
                                            IMediaSample* pMediaSample)
{
    switch (nEventCode)
            {
            case IPinEventSink::PE_MediaSampleReceived:
                // a new image was received so the processing is started
                if (pSource == &m_oPinInputVideo)
                {                
                    UCOM_TIMING_SPOT(m_oProcessStart);
                    ProcessVideo(pMediaSample);        
                    UCOM_TIMING_SPOT(m_oProcessEnd);
                }
                if(pSource == &maneuverid_Jury)
                {
                	tUInt32 timeStamp = 0;
        			cObjectPtr<IMediaCoder> pCoderInput;
        			RETURN_IF_FAILED(mediatype_maneuverid_Jury->Lock(pMediaSample, &pCoderInput));
        			pCoderInput->Get("f32Value", (tVoid*)&ManeuverJuryID);
        			LOG_INFO(cString::Format("Marker Detection - Maneuver from Jury received ID: %f",ManeuverJuryID));
        			pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
        			mediatype_maneuverid_Jury->Unlock(pCoderInput);
                }
                if(pSource == &maneuver_finish)
                {
                	tUInt32 timeStamp = 0;
        			cObjectPtr<IMediaCoder> pCoderInput;
        			RETURN_IF_FAILED(mediatype_maneuver_finish->Lock(pMediaSample, &pCoderInput));
        			pCoderInput->Get("bValue", (tVoid*)&var_finish_flag);
        			if(var_finish_flag)
        				LOG_INFO(cString::Format("Marker Detection - Maneuver Finish received"));
        			//flagtoreducespeed = tFalse;
        			pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
        			mediatype_maneuver_finish->Unlock(pCoderInput);
                }
                break;
            case IPinEventSink::PE_MediaTypeChanged:
                if (pSource == &m_oPinInputVideo)
                {
                    //the input format was changed, so the imageformat has to changed in this filter also
                    cObjectPtr<IMediaType> pType;
                    RETURN_IF_FAILED(m_oPinInputVideo.GetMediaType(&pType));

                    cObjectPtr<IMediaTypeVideo> pTypeVideo;
                    RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));

                    UpdateInputImageFormat(m_oPinInputVideo.GetFormat());    
                    UpdateOutputImageFormat(m_oPinInputVideo.GetFormat());                
                }            
                break;
            default:
                break;
        }    
    RETURN_NOERROR;
}

tResult cMarkerStraightFilter::ProcessVideo(adtf::IMediaSample* pISample)
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
    Mat input_leftimage;
    //receiving data from input sample, and saving to TheInputImage
    if (IS_OK(pISample->Lock(&l_pSrcBuffer)))
        {
        //convert to mat
        TheInputImage  = Mat(m_sInputFormat.nHeight,m_sInputFormat.nWidth,CV_8UC3,(tVoid*)l_pSrcBuffer,m_sInputFormat.nBytesPerLine);    
        
        input_leftimage = TheInputImage(Range(row1,row2),Range(col1,col2)).clone();
        //imshow("marker",input_leftimage); waitKey(1);
        // doing the detection of markers in image
        // detect image only on the left part side
        m_TheMarkers.clear();
        m_MDetector.detect(input_leftimage,m_TheMarkers_org,m_TheCameraParameters,static_cast<float>(m_f32MarkerSize));
        }
    pISample->Unlock(l_pSrcBuffer); 

    // Vinoth : Check if the id is proper by locating the line below the image
    // if line is not present then remove the marker from the vector
    for (unsigned int i=0;i<m_TheMarkers_org.size();i++)
    {
    	//c++ cv::Mat inputImage; ... vector< int > markerIds; vector< vector<Point2f> > markerCorners
    	// get the center point of the marker

    	if((fabs(MANEUVER_INTERSECTION_STRAIGHT - ManeuverJuryID) < FLT_EPSILON))
    	{
    		//flagtoreducespeed = tTrue;
			if(var_finish_flag && (MARKER_ID_UNMARKEDINTERSECTION == m_TheMarkers_org[i].id || MARKER_ID_STOPANDGIVEWAY == m_TheMarkers_org[i].id
					|| MARKER_ID_HAVEWAY == m_TheMarkers_org[i].id  || MARKER_ID_GIVEWAY == m_TheMarkers_org[i].id )
					&&(m_TheMarkers_org[i].getArea() > f32RequiredArea))  //if maneuvers is finished then only go for next start
			{
				Point2f points = m_TheMarkers_org[i].getCenter();
				Mat process_line_ver = TheInputImage(Range((points.y-row1_ver)>0? points.y-row1_ver:0 ,(points.y+row2_ver)<480? points.y+row2_ver:480),
				   Range((points.x+col1_ver)<640?(points.x+col1_ver):640,(points.x+col2_ver)<640?points.x+col2_ver:640)).clone();
				if(imshowflag_ver)
				{
					imshow("processline_ver",process_line_ver);
					waitKey(1);

				}
				Mat grey_ver,greythresh_ver,canny_ver;
				vector<Vec2f> ver_lines;
				GaussianBlur(process_line_ver, process_line_ver, Size(11,11), 0, 0, BORDER_DEFAULT); // Filter
				cvtColor(process_line_ver ,grey_ver,CV_BGR2GRAY);// Grey Image
				threshold(grey_ver, greythresh_ver, m_nThresholdValue, 500,THRESH_BINARY);// Generate Binary Image
				Canny(greythresh_ver, canny_ver, 0, 2, 3, false);// Detect Edges
				HoughLines(canny_ver,ver_lines,1,CV_PI/180,40,0,0);
				if(imshowflag_ver)
				{
					imshow("cannyimage_ver",canny_ver);
					waitKey(1);
				}

				Mat process_line = TheInputImage(Range((points.y+row1_l)<480? points.y+row1_l:480 ,(points.y+row2_l)<480? points.y+row2_l:480),
				   Range((points.x-col1_l)>0?(points.x-col1_l):0,(points.x+col2_l)<640?points.x+col2_l:640)).clone();
				if(imshowflag)
				{
					imshow("processline_hor",process_line);
					waitKey(1);
				}
				Mat grey,greythresh,canny;
				vector<Vec2f> lines;
				GaussianBlur(process_line, process_line, Size(11,11), 0, 0, BORDER_DEFAULT); // Filter
				cvtColor(process_line, grey ,CV_BGR2GRAY);// Grey Image
				threshold(grey, greythresh, m_nThresholdValue, 500,THRESH_BINARY);// Generate Binary Image
				Canny(greythresh, canny, 0, 2, 3, false);// Detect Edges
				HoughLines(canny,lines,1,CV_PI/180,20,0,0);
				if(imshowflag)
				{
					imshow("cannyimag_hor",canny);
					waitKey(1);
				}
				int count = 0,ver_count = 0;
				for(int i=0;i<ver_lines.size();i++)
				{
					if(ver_lines[i][1] > 0.7 && ver_lines[i][1] < 1.22)
						ver_count++;
				}
				for(int i=0;i<lines.size();i++)
				{
					if(lines[i][1] > 1.3 && lines[i][1] < 1.6)
						count++;
				}
				if((count >=count_lines_marker)||(ver_count >=count_lines_marker) )
				{
					LOG_INFO(cString::Format("Marker Detection hor_lines detected %d ver_lines detected %d , start Maneuver id:%f",count,ver_count,var_maneuver_id));
					FrameCount++;
				}
				if((FrameCount))// &&(!manuever_enable_flag))
				{
					FrameCount = 0;
					var_finish_flag = tFalse;
					LOG_INFO(cString::Format("Marker Detection maneuverstart sent detected"));
					tTimeStamp timeStamp = _clock->GetStreamTime();
					cObjectPtr<IMediaSample> pMediaSamplemaneuverstart;
					AllocMediaSample((tVoid**)&pMediaSamplemaneuverstart);
					cObjectPtr<IMediaSerializer>pSerializermaneuverstart;
					mediatype_maneuver_id->GetMediaSampleSerializer(&pSerializermaneuverstart);
					tInt nSizemaneuverstart = pSerializermaneuverstart->GetDeserializedSize();
					pMediaSamplemaneuverstart->AllocBuffer(nSizemaneuverstart);
					cObjectPtr<IMediaCoder> pCoderOutputmaneuverleft;
					mediatype_maneuver_id->WriteLock(pMediaSamplemaneuverstart, &pCoderOutputmaneuverleft);
					pCoderOutputmaneuverleft->Set("f32Value", (tVoid*)&(ManeuverJuryID)); //ManeuverJuryID
					ManeuverJuryID = MANEUVER_INTERSECTION_DEFAULT;
					pCoderOutputmaneuverleft->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
					mediatype_maneuver_id->Unlock(pCoderOutputmaneuverleft);
					pMediaSamplemaneuverstart->SetTime(_clock->GetStreamTime());
					maneuver_id.Transmit(pMediaSamplemaneuverstart);
					//for testing
					global_i++;
					if (global_i == 9)
					{
						//transmit the output
						cObjectPtr<IMediaSample> pMediaSampleheadlight;
						AllocMediaSample((tVoid**)&pMediaSampleheadlight);
						cObjectPtr<IMediaSerializer> pSerializereheadlight;
					    med_typ_m_headlight->GetMediaSampleSerializer(&pSerializereheadlight);
						tInt nSizeheadlight = pSerializereheadlight->GetDeserializedSize();
						pMediaSampleheadlight->AllocBuffer(nSizeheadlight);
						cObjectPtr<IMediaCoder> pCoderOutputheadlight;
						med_typ_m_headlight->WriteLock(pMediaSampleheadlight, &pCoderOutputheadlight);
						tBool local_var = tTrue;
						pCoderOutputheadlight->Set("bValue", (tVoid*)&(local_var));
						pCoderOutputheadlight->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
						med_typ_m_headlight->Unlock(pCoderOutputheadlight);
						pMediaSampleheadlight->SetTime(_clock->GetStreamTime());
						m_headlight.Transmit(pMediaSampleheadlight);
						LOG_INFO("Marker straight reached finished");
						global_i = (int)0;
					}
					var_maneuver_id = TestArray[global_i];
				}
			}
    	}
   		m_TheMarkers.push_back(m_TheMarkers_org[i]);
    }
    // 1: nothing is drawn, 2: results are drawn so we need the have copy of the frame otherwise the orginal mediasample is modified
    if (m_iOutputMode==2) 
    {  
        // do a deep copy of the image, otherwise the orginal frame is modified
        TheOutputImage = TheInputImage.clone();
    }
   
    //print marker info and draw the markers in image    
    for (unsigned int i=0;i<m_TheMarkers.size();i++) 
        { 
        if (m_iOutputMode==2) 
            {                
                // draw the marker in the image
                m_TheMarkers[i].draw(TheOutputImage,Scalar(0,0,255),1);
                
                // draw cube in image
                //CvDrawingUtils::draw3dCube(TheInputImage,m_TheMarkers[i],m_TheCameraParameters);
                
                // draw 3d axis in image if the intrinsic params were loaded
                if (m_bCamaraParamsLoaded) CvDrawingUtils::draw3dAxis(TheOutputImage,m_TheMarkers[i],m_TheCameraParameters);                
            }
        // call the function to transmit a road sign sample with the detected marker
        sendRoadSignStruct(static_cast<tInt16>(m_TheMarkers[i].id), m_TheMarkers[i].getArea(),pISample->GetTime());
        
        // call the function to transmit a extended road sign sample with the detected marker if the Tvec in the marker was correctly set
        if (m_bCamaraParamsLoaded) 
            {
                sendRoadSignStructExt(static_cast<tInt16>(m_TheMarkers[i].id), m_TheMarkers[i].getArea(),pISample->GetTime(),m_TheMarkers[i].Tvec, m_TheMarkers[i].Rvec);
                //get camera position

                if (m_TheMarkers[i].id == 2) 
                {
                    cv::Mat rotationVector( m_TheMarkers[i].Rvec);;
                    cv::Mat translationVector(m_TheMarkers[i].Tvec);;
                    tFloat32 aSignPosition[3];
                    aSignPosition[0] = 0;
                    aSignPosition[1] = 0;
                    aSignPosition[2] = 0;
                    vector<Point3d> m_worldCoordinatesSign;
                    tFloat32 halfSize = m_f32MarkerSize/2;
                    m_worldCoordinatesSign.push_back(Point3d(aSignPosition[0]-halfSize,aSignPosition[1] - halfSize,aSignPosition[2]));
                    m_worldCoordinatesSign.push_back(Point3d(aSignPosition[0]+halfSize,aSignPosition[1] - halfSize,aSignPosition[2]));
                    m_worldCoordinatesSign.push_back(Point3d(aSignPosition[0]+halfSize,aSignPosition[1] + halfSize,aSignPosition[2]));
                    m_worldCoordinatesSign.push_back(Point3d(aSignPosition[0]-halfSize,aSignPosition[1] + halfSize,aSignPosition[2]));
                                      
                    
        

                   vector<Point2d> ImagePoints;
                    for (int c = 0; c < 4; c++) {   
                        
                        //LOG_INFO(cString::Format("Imagepoint %d: %f,%f",c,m_TheMarkers[i][c].x,m_TheMarkers[i][c].y));
                        ImagePoints.push_back(Point2d(m_TheMarkers[i][c].x,m_TheMarkers[i][c].y));
                    }


                    cv::solvePnP(m_worldCoordinatesSign,ImagePoints,m_Intrinsics,m_Distorsion,rotationVector,translationVector,true,0);

                    cv::Mat RodriguesMat;
                    
                    cv::Rodrigues(rotationVector,RodriguesMat);
                    RodriguesMat = RodriguesMat.t();

                    translationVector = -RodriguesMat * translationVector;

                    /*cv::Rodrigues(Rodrigues.t(),cameraRotationVector);
                    cv::Mat t= translationVector.t();
                    cameraTranslationVector= -cameraRotationVector * t;*/                 
                    
                }
            }

        
    }
    
    //update new media sample with image data if something has to be transmitted
    if (m_iOutputMode==2) 
        {
        pNewSample->Update(pISample->GetTime(), TheOutputImage.data, m_sOutputFormat.nSize, 0);
        m_oPinOutputVideo.Transmit(pNewSample);    
        }    

    RETURN_NOERROR;
}

tResult cMarkerStraightFilter::UpdateInputImageFormat(const tBitmapFormat* pFormat)
{
    if (pFormat != NULL)
    {
        m_sInputFormat = (*pFormat);
        
        LOG_INFO(adtf_util::cString::Format("Marker Detection Filter: Input: Size %d x %d ; BPL %d ; Size %d , PixelFormat; %d", m_sInputFormat.nWidth,m_sInputFormat.nHeight,m_sInputFormat.nBytesPerLine, m_sInputFormat.nSize, m_sInputFormat.nPixelFormat));        
        
    }
    
    RETURN_NOERROR;
}

tResult cMarkerStraightFilter::UpdateOutputImageFormat(const tBitmapFormat* pFormat)
{
    if (pFormat != NULL)
    {
        m_sOutputFormat = (*pFormat);    
        
        LOG_INFO(adtf_util::cString::Format("Marker Detection Filter: Output: Size %d x %d ; BPL %d ; Size %d, PixelFormat; %d", m_sOutputFormat.nWidth,m_sOutputFormat.nHeight,m_sOutputFormat.nBytesPerLine, m_sOutputFormat.nSize, m_sOutputFormat.nPixelFormat));
        
        m_oPinOutputVideo.SetFormat(&m_sOutputFormat, NULL);                
    }
    
    RETURN_NOERROR;
}

tResult cMarkerStraightFilter::sendRoadSignStruct(const tInt16 &i16ID, const tFloat32 &f32MarkerSize, const tTimeStamp &timeOfFrame)
{    
    // create new media sample
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));
    
    // get the serializer
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionRoadSign->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    
    // alloc the buffer memory
    RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize));

    {   // focus for sample write lock
        //write date to the media sample with the coder of the descriptor
        __adtf_sample_write_lock_mediadescription(m_pDescriptionRoadSign,pMediaSample,pCoder);
        
        // get IDs
        if(!m_bIDsRoadSignSet)
        {
            pCoder->GetID("i16Identifier",m_szIDRoadSignI16Identifier);
            pCoder->GetID("f32Imagesize", m_szIDRoadSignF32Imagesize);
            m_bIDsRoadSignSet = tTrue;
        }  

        pCoder->Set(m_szIDRoadSignI16Identifier, (tVoid*)&i16ID);
        pCoder->Set(m_szIDRoadSignF32Imagesize, (tVoid*)&f32MarkerSize);       

        pMediaSample->SetTime(timeOfFrame);
    }
    
    //doing the transmit
    RETURN_IF_FAILED(m_oPinRoadSign.Transmit(pMediaSample));
    
    //print debug info if activated
    if (m_bDebugModeEnabled)  LOG_INFO(cString::Format("Zeichen ID %d erkannt. Area: %f",i16ID,f32MarkerSize));
    
    RETURN_NOERROR;
}

tResult cMarkerStraightFilter::sendRoadSignStructExt(const tInt16 &i16ID, const tFloat32 &f32MarkerSize, const tTimeStamp &timeOfFrame, const Mat &Tvec, const Mat &Rvec)
{    
    // create new media sample
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));
    
    // get the serializer
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionRoadSignExt->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    
    // alloc the buffer memory
    RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize));

    {   // focus for sample write lock
        //write date to the media sample with the coder of the descriptor
        __adtf_sample_write_lock_mediadescription(m_pDescriptionRoadSignExt,pMediaSample,pCoder);
        
        // get IDs
        if(!m_bIDsRoadSignExtSet)
        {
            pCoder->GetID("i16Identifier",m_szIDRoadSignExtI16Identifier);
            pCoder->GetID("f32Imagesize", m_szIDRoadSignExtF32Imagesize);
            pCoder->GetID("af32RVec[0]", m_szIDRoadSignExtAf32RVec);
            pCoder->GetID("af32TVec[0]", m_szIDRoadSignExtAf32TVec);
            m_bIDsRoadSignExtSet = tTrue;
        }  
        
        pCoder->Set(m_szIDRoadSignExtI16Identifier, (tVoid*)&i16ID);
        pCoder->Set(m_szIDRoadSignExtF32Imagesize, (tVoid*)&f32MarkerSize);
        pCoder->Set("af32TVec", (tVoid*)Tvec.data);
        pCoder->Set("af32RVec", (tVoid*)Rvec.data);       

        pMediaSample->SetTime(timeOfFrame);
    }
    //doing the transmit
    RETURN_IF_FAILED(m_oPinRoadSignExt.Transmit(pMediaSample));
    
    //print debug info if activated
    if (m_bDebugModeEnabled) LOG_INFO(cString::Format("Zeichen ID %d erkannt,TVec: %f, %f, %f", i16ID, Tvec.at<float>(0),  Tvec.at<float>(1),  Tvec.at<float>(2)));
    RETURN_NOERROR;
}

