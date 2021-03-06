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


// arduinofilter.cpp : Definiert die exportierten Funktionen f�r die DLL-Anwendung.
//
#include "stdafx.h"
#include "cDriverFilter.h"

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

//#define MIN_DISTANCE "cdriverfilter::threshold_value"

ADTF_FILTER_PLUGIN("AADC DRIVER FILTER", OID_ADTF_DRIVERFILTER, cdriverfilter)

cdriverfilter::cdriverfilter(const tChar* __info) : cFilter(__info)
{

	SetPropertyStr("ManeuverFile", "");
	SetPropertyBool("ManeuverFile" NSSUBPROP_FILENAME, tTrue);
	SetPropertyStr("ManeuverFile" NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "XML Files (*.xml)");

first_sample_ready =true;
	timeflag = tFalse;
	time_diff = 5000000;
	setstateready=false;
	setstatecomplete=false;
	
	var_Emergency_Brake_Flag = tFalse;
    var_accelerate = 1.0f;
    
   
    completion = false;
    maneuverCounter = 0;
    first_sample=false;
}

cdriverfilter::~cdriverfilter()
{
	
}

tResult cdriverfilter::Init(tInitStage eStage, __exception)
{
	RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))
		if (eStage == StageFirst)
		{
			// create description manager
			cObjectPtr<IMediaDescriptionManager> pDescManager;
			RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**)&pDescManager, __exception_ptr));

			// get media type for input pins
			tChar const * med_des_Emergency_Brake_Flag = pDescManager->GetMediaDescription("tBoolSignalValue");
			RETURN_IF_POINTER_NULL(med_des_Emergency_Brake_Flag);
			cObjectPtr<IMediaType> med_typ_Emergency_Brake_Flag_loc	= new cMediaType(0, 0, 0, "tBoolSignalValue", med_des_Emergency_Brake_Flag, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
			RETURN_IF_FAILED(med_typ_Emergency_Brake_Flag_loc->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&med_typ_Emergency_Brake_Flag));
			RETURN_IF_FAILED(Emergency_Brake_Flag.Create("start_signalid", med_typ_Emergency_Brake_Flag_loc, static_cast<IPinEventSink*> (this)));
			RETURN_IF_FAILED(RegisterPin(&Emergency_Brake_Flag));

			tChar const * strDescSignalUltrasonic_Front_Center = pDescManager->GetMediaDescription("tSignalValue");
			RETURN_IF_POINTER_NULL(strDescSignalUltrasonic_Front_Center);
			cObjectPtr<IMediaType> pTypeSignalUltrasonic_Front_Center = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalUltrasonic_Front_Center, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
			RETURN_IF_FAILED(pTypeSignalUltrasonic_Front_Center->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescUltrasonic_Front_Center));
			RETURN_IF_FAILED(m_Ultrasonic_Front_Center.Create("ultrasonic_front_center", pTypeSignalUltrasonic_Front_Center, static_cast<IPinEventSink*> (this)));
			RETURN_IF_FAILED(RegisterPin(&m_Ultrasonic_Front_Center));
			
			tChar const * med_des_cur_man_complete_Flag = pDescManager->GetMediaDescription("tBoolSignalValue");
			RETURN_IF_POINTER_NULL(med_des_cur_man_complete_Flag);
			cObjectPtr<IMediaType> med_typ_cur_man_complete_Flag_loc	= new cMediaType(0, 0, 0, "tBoolSignalValue", med_des_cur_man_complete_Flag, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
			RETURN_IF_FAILED(med_typ_cur_man_complete_Flag_loc->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&med_typ_cur_man_complete_Flag));
			RETURN_IF_FAILED(cur_man_complete_Flag.Create("Cur_Man_Finish", med_typ_cur_man_complete_Flag_loc, static_cast<IPinEventSink*> (this)));
			RETURN_IF_FAILED(RegisterPin(&cur_man_complete_Flag));
			
			//input drive_struct 
			tChar const * strDescdrive_struct = pDescManager->GetMediaDescription("tDriverStruct");
			RETURN_IF_POINTER_NULL(strDescdrive_struct);
			cObjectPtr<IMediaType> pTypedrive_struct = new cMediaType(0, 0, 0, "tDriverStruct", strDescdrive_struct, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
			RETURN_IF_FAILED(pTypedrive_struct->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&mediatype_drive_struct));
			RETURN_IF_FAILED(drive_struct.Create("Drive_Struct", pTypedrive_struct, static_cast<IPinEventSink*> (this)));
			RETURN_IF_FAILED(RegisterPin(&drive_struct));

			//output pins
			tChar const * med_des_m_readystate = pDescManager->GetMediaDescription("tBoolSignalValue");
			RETURN_IF_POINTER_NULL(med_des_m_readystate);
			cObjectPtr<IMediaType> pTypereadystate_struct	= new cMediaType(0, 0, 0, "tBoolSignalValue", med_des_m_readystate, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
			RETURN_IF_FAILED(pTypereadystate_struct->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&med_typ_readystate));
			RETURN_IF_FAILED(m_readystate.Create("Ready_State", pTypereadystate_struct, static_cast<IPinEventSink*> (this)));
			RETURN_IF_FAILED(RegisterPin(&m_readystate));
			
			tChar const * med_des_m_completestate = pDescManager->GetMediaDescription("tBoolSignalValue");
			RETURN_IF_POINTER_NULL(med_des_m_completestate);
			cObjectPtr<IMediaType> pTypecompletestate_struct	= new cMediaType(0, 0, 0, "tBoolSignalValue", med_des_m_completestate, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
			RETURN_IF_FAILED(pTypecompletestate_struct->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&med_typ_completestate));
			RETURN_IF_FAILED(m_completestate.Create("complete_State", pTypecompletestate_struct, static_cast<IPinEventSink*> (this)));
			RETURN_IF_FAILED(RegisterPin(&m_completestate));
			
			tChar const * med_des_m_maneuverid = pDescManager->GetMediaDescription("tSignalValue");
			RETURN_IF_POINTER_NULL(med_des_m_maneuverid);
			cObjectPtr<IMediaType> pTypemaneuverid_struct	= new cMediaType(0, 0, 0, "tSignalValue", med_des_m_maneuverid, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
			RETURN_IF_FAILED(pTypemaneuverid_struct->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&mediatype_maneuverid));
			RETURN_IF_FAILED(m_maneuverid.Create("Maneuver_ID", pTypemaneuverid_struct, static_cast<IPinEventSink*> (this)));
			RETURN_IF_FAILED(RegisterPin(&m_maneuverid));

			
		}
		else if (eStage == StageNormal)
		{

		}
		else if (eStage == StageGraphReady)
		{
               loadManeuverList();
		}
		RETURN_NOERROR;

}

		tResult cdriverfilter::Start(__exception)
		{
			return cFilter::Start(__exception_ptr);
		}

		tResult cdriverfilter::Stop(__exception)
		{
			return cFilter::Stop(__exception_ptr);
		}

		tResult cdriverfilter::Shutdown(tInitStage eStage, __exception)
		{
			if (eStage == StageNormal)
			{
				 
			}
			return cFilter::Shutdown(eStage, __exception_ptr);
		}


tResult cdriverfilter::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{

	RETURN_IF_POINTER_NULL(pMediaSample);
	RETURN_IF_POINTER_NULL(pSource);
	if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
	{
		tUInt32 timeStamp = 0;	tInt8 i8StateID = 0; tInt16 i16ManeuverEntry = 0; 
		if(pSource == &drive_struct)
		{
			
		                __adtf_sample_read_lock_mediadescription(mediatype_drive_struct,pMediaSample,pCoderInput);       
		                
		                // get IDs
		                if(!m_bDriver)
		                {
		                    pCoderInput->GetID("i8Identifier",m_szIDDriverI8);
		                    pCoderInput->GetID("i16Identifier", m_szIDDriverI16);
		                    m_bDriver = tTrue;
		                }  

		                // get the values from sample
		                pCoderInput->Get(m_szIDDriverI8, (tVoid*)&i8StateID);
		                pCoderInput->Get(m_szIDDriverI16, (tVoid*)&i16ManeuverEntry);
                          //LOG_INFO(cString::Format("i8StateID = %d, i16ManeuverEntry = %d, ", i8StateID, i16ManeuverEntry));
		               
					 if(0 == i8StateID && first_sample_ready)
                              {
						 start_time = _clock->GetStreamTime();
		                	timeflag = tTrue; first_sample_ready=false;
                               }
		                process (start_time, i8StateID, i16ManeuverEntry);
		                
		}

else if (pSource == &m_Ultrasonic_Front_Center)
{
		if (timeflag)
		{
			end_time = _clock->GetStreamTime();
			if(time_diff < (end_time - start_time)) //3<0 ....3<3.2
			{
				timeflag = false;
				setstateready=true;
				
				cObjectPtr<IMediaSample> pMediaSamplereadystate;
				AllocMediaSample((tVoid**)&pMediaSamplereadystate);
				cObjectPtr<IMediaSerializer>pSerializerreadystate;
				med_typ_readystate->GetMediaSampleSerializer(&pSerializerreadystate);
				tInt nSizereadystate = pSerializerreadystate->GetDeserializedSize();
				pMediaSamplereadystate->AllocBuffer(nSizereadystate);
				cObjectPtr<IMediaCoder> pCoderOutputreadystate;
				med_typ_readystate->WriteLock(pMediaSamplereadystate, &pCoderOutputreadystate);
                    LOG_INFO(cString::Format("setstateready is sent"));
				pCoderOutputreadystate->Set("bValue", (tVoid*)&(setstateready));
				pCoderOutputreadystate->Set("ui32ArduinoTimestamp", (tVoid*)&start_time);
				med_typ_readystate->Unlock(pCoderOutputreadystate);
				pMediaSamplereadystate->SetTime(_clock->GetStreamTime());
				m_readystate.Transmit(pMediaSamplereadystate);
			}
			
		}

}
		
		else if (pSource == &cur_man_complete_Flag)
		{
			//LOG_INFO("Vinoth distance info");
			cObjectPtr<IMediaCoder> pCoderInput;
			RETURN_IF_FAILED(med_typ_cur_man_complete_Flag->Lock(pMediaSample, &pCoderInput));
			pCoderInput->Get("bValue", (tVoid*)&completion);
               //LOG_INFO(cString::Format("completion flag received"));
			pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
			med_typ_cur_man_complete_Flag->Unlock(pCoderInput);	
	    }
		            
		else 
		{
			RETURN_ERROR(ERR_FAILED);
		}
	}
	RETURN_NOERROR;
}

tResult cdriverfilter:: process(tTimeStamp start_time, tInt8 action_id, tInt16 maneuver_id)
{
	//tFloat32 f32maneuver_id = maneuver_id;
	
	
	if (action_id == 0) // wait for 5 Seconds and then send READY
	{
		first_sample =false;	

	}
	
    else if(action_id == 1) //action_START, start the maneuver sequence 
	{
    	   
    	if (!first_sample) 
    		{
    		     maneuverCounter = maneuver_id;
    		     first_sample=true;
    		}
    	
    	if (completion) 
    	{ 
    		  maneuverCounter++;
    		  completion = false;
    	}
    		  
    	 if (maneuverCounter >= allTheManeuversInARow.size())
    	    	{
    	    		setstatecomplete = true;
    	    		
    	    		cObjectPtr<IMediaSample> pMediaSamplecompletestate;
    	    		AllocMediaSample((tVoid**)&pMediaSamplecompletestate);
    	    		cObjectPtr<IMediaSerializer>pSerializercompletestate;
    	    		med_typ_completestate->GetMediaSampleSerializer(&pSerializercompletestate);
    	    		tInt nSizecompletestate = pSerializercompletestate->GetDeserializedSize();
    	    		pMediaSamplecompletestate->AllocBuffer(nSizecompletestate);
    	    		cObjectPtr<IMediaCoder> pCoderOutputcompletestate;
    	    		med_typ_completestate->WriteLock(pMediaSamplecompletestate, &pCoderOutputcompletestate);
    	    		pCoderOutputcompletestate->Set("bValue", (tVoid*)&(setstatecomplete));
    	    		pCoderOutputcompletestate->Set("ui32ArduinoTimestamp", (tVoid*)&start_time);
    	    		med_typ_completestate->Unlock(pCoderOutputcompletestate);
    	    		pMediaSamplecompletestate->SetTime(_clock->GetStreamTime());
    	    		m_completestate.Transmit(pMediaSamplecompletestate);
    	    	}
    	    
        	else if (allTheManeuversInARow[maneuverCounter].action.IsEqual("left"))    nextManeuver = MANEUVER_INTERSECTION_LEFT ;
            else if (allTheManeuversInARow[maneuverCounter].action.IsEqual("right"))  nextManeuver = MANEUVER_INTERSECTION_RIGHT;
        	else if (allTheManeuversInARow[maneuverCounter].action.IsEqual("straight")) nextManeuver = MANEUVER_INTERSECTION_STRAIGHT;
        	else if (allTheManeuversInARow[maneuverCounter].action.IsEqual("parallel_parking")) nextManeuver = MANEUVER_PARALLEL_PARKING;
        	else if (allTheManeuversInARow[maneuverCounter].action.IsEqual("cross_parking")) nextManeuver = MANEUVER_CROSS_PARKING;
            else if (allTheManeuversInARow[maneuverCounter].action.IsEqual("pull_out_left")) nextManeuver = START_PULLOVER_LEFT;
            else if (allTheManeuversInARow[maneuverCounter].action.IsEqual("pull_out_right"))
            {
        	            if (maneuverCounter != 0)
        		  			{
        	                    if (allTheManeuversInARow[maneuverCounter-1].action.IsEqual("parallel_parking"))
        		        	    nextManeuver = START_PULLOVER_PARALLEL;
        	                    else if (allTheManeuversInARow[maneuverCounter-1].action.IsEqual("cross_parking"))
        		        		nextManeuver = START_PULLOVER_RIGHT;
        	                }
        	            else
        		                nextManeuver = START_PULLOVER_RIGHT; // cross (to change during car setup if this assumption is wrong)
            }
    		tTimeStamp timeStamp = _clock->GetStreamTime();
    		cObjectPtr<IMediaSample> pMediaSamplemaneuverid;
    		AllocMediaSample((tVoid**)&pMediaSamplemaneuverid);
    		cObjectPtr<IMediaSerializer>pSerializermaneuverid;
    		mediatype_maneuverid->GetMediaSampleSerializer(&pSerializermaneuverid);
    		tInt nSizemaneuverid = pSerializermaneuverid->GetDeserializedSize();
    		pMediaSamplemaneuverid->AllocBuffer(nSizemaneuverid);
    		cObjectPtr<IMediaCoder> pCoderOutputmaneuverid;
    		mediatype_maneuverid->WriteLock(pMediaSamplemaneuverid, &pCoderOutputmaneuverid);
          LOG_INFO(cString::Format("Next Maneuver", nextManeuver));
    		pCoderOutputmaneuverid->Set("f32Value", (tVoid*)&(nextManeuver));
    		pCoderOutputmaneuverid->Set("ui32ArduinoTimestamp", (tVoid*)&start_time);
    		mediatype_maneuverid->Unlock(pCoderOutputmaneuverid);
    		pMediaSamplemaneuverid->SetTime(_clock->GetStreamTime());
    		m_maneuverid.Transmit(pMediaSamplemaneuverid);
    		
    }	
}
tResult cdriverfilter::loadManeuverList()
{

	m_maneuverListFile = GetPropertyStr("ManeuverFile");

	if (m_maneuverListFile.IsEmpty())
	{
		LOG_ERROR("DriverFilter: Maneuver file not found");
		RETURN_ERROR(ERR_INVALID_FILE);
	}

	ADTF_GET_CONFIG_FILENAME(m_maneuverListFile);

	m_maneuverListFile = m_maneuverListFile.CreateAbsolutePath(".");

	//Load file, parse configuration, print the data

	if (cFileSystem::Exists(m_maneuverListFile))
	{
		cDOM oDOM;
		oDOM.Load(m_maneuverListFile);
		cDOMElementRefList oSectorElems;
		cDOMElementRefList oManeuverElems;

		//read first Sector Elem
		if (IS_OK(oDOM.FindNodes("AADC-Maneuver-List/AADC-Sector", oSectorElems)))
		{
			//iterate through sectors
			for (cDOMElementRefList::iterator itSectorElem = oSectorElems.begin(); itSectorElem != oSectorElems.end(); ++itSectorElem)
			{
				//if sector found
				tSector sector;
				sector.id = (*itSectorElem)->GetAttributeUInt32("id");

				if (IS_OK((*itSectorElem)->FindNodes("AADC-Maneuver", oManeuverElems)))
				{
					//iterate through maneuvers
					for (cDOMElementRefList::iterator itManeuverElem = oManeuverElems.begin(); itManeuverElem != oManeuverElems.end(); ++itManeuverElem)
					{
						tAADC_Maneuver man;
						man.id = (*itManeuverElem)->GetAttributeUInt32("id");
						man.action = (*itManeuverElem)->GetAttribute("action");
						sector.maneuverList.push_back(man);
                                                allTheManeuversInARow.push_back(man);
					}
				}

				m_sectorList.push_back(sector);
			}
		}
		if (oSectorElems.size() > 0)
		{
			LOG_INFO("DriverFilter: Loaded Maneuver file successfully.");
		}
		else
		{
			LOG_ERROR("DriverFilter: no valid Maneuver Data found!");
			RETURN_ERROR(ERR_INVALID_FILE);
		}
	}
	else
	{
		LOG_ERROR("DriverFilter: no valid Maneuver File found!");
		RETURN_ERROR(ERR_INVALID_FILE);
	}


	//checks if data are valid
	//RETURN_IF_FAILED(CheckConfigurationData());

	RETURN_NOERROR;
}
