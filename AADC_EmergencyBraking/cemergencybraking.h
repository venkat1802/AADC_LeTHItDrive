/**
Copyright (c) 
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: “This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.”
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: spiesra $  $Date:: 2015-05-13 08:29:07#$ $Rev:: 35003   $
**********************************************************************/

#pragma once

#ifndef _EMERGENCYBRAKING_H_
#define _EMERGENCYBRAKING_H_

#include "stdafx.h"

#define OID_ADTF_EMERGENCYBRAKING "adtf.aadc.cemergencybraking"

/*!
This filter implements a controller to set the wheelspeed of the vehicle with a P/PI/PID or PT1 algorithm. The input pin measured_wheelSpeed  has to be connected to the output pin of the Converter Wheels Filter and the desired wheel speed has to be set to the input pin set_WheelSpeed. 
The controller parameters have to be adapted to each individual car. The default values for the PT1 controller are good start for the controller but maybe have to be adapted to the individual team car. There are different methods to get the correct parameter, please refer to other literature.
The output pin actuator_output have to be connected to a Calibration XML which maps the speed in m/〖sec〗^2  to servo angle of steering controller. In experiments the following mapping was found:

This values are saved in the Sample XML SpeedController.xml in the folder configuration_files and can be used at the beginning.
A typical configuration with the Wheel Speed Controller should contain at least the following Filters:
 
*/
class cemergencybraking : public adtf::cFilter
{
    //ADTF_DECLARE_FILTER_VERSION(OID_ADTF_EMERGENCYBRAKING, "AADC Emergency Braking", OBJCAT_DataFilter);    
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_EMERGENCYBRAKING, "AADC Emergency Braking", OBJCAT_DataFilter, "Emergency Braking", 1, 0,0, "Beta Version");    

    cInputPin m_Ultrasonic_Front_Left;                
    cInputPin m_Ultrasonic_Front_Center;
    cInputPin m_Ultrasonic_Front_Right;    
    cInputPin m_steer_input;

    cOutputPin m_emergencybrake;	
	
    cObjectPtr<IMediaTypeDescription> m_pDescUltrasonic_Front_Left;
    cObjectPtr<IMediaTypeDescription> m_pDescUltrasonic_Front_Center;
    cObjectPtr<IMediaTypeDescription> m_pDescUltrasonic_Front_Right;
    cObjectPtr<IMediaTypeDescription> m_pDescm_steer_input;

    cObjectPtr<IMediaTypeDescription> m_pDescemergencybrake;


public:
    cemergencybraking(const tChar* __info);
    virtual ~cemergencybraking();
    tResult PropertyChanged(const char* strProperty);

protected: // overwrites cFilter
    tResult Init(tInitStage eStage, __exception);
    tResult Start(__exception = NULL);
    tResult Stop(__exception = NULL);
    tResult Shutdown(tInitStage eStage, __exception);        
    tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);


private:    
   // tResult CreatePins( __exception = NULL);
    //tResult CreateOutputPins( __exception = NULL);

	tResult ReadProperties(const tChar* strPropertyName);

    tFloat32 Ultrasonic_Front_Left;
    tFloat32 Ultrasonic_Front_Center;
    tFloat32 Ultrasonic_Front_Right;

	tFloat32 var_steer;
	tBool emergencybrake;

	tFloat32    thresholdlimit;

	tResult ebfunction(tUInt32 timeStamp);
	tResult generateop(tUInt32 timeStamp);

};

#endif

