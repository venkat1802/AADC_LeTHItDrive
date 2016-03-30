/**
 *
 * ADTF Demo Source.
 *
 * @file
 * Copyright &copy; Audi Electronics Venture GmbH. All rights reserved
 *
 * $Author: spiesra $
 * $Date: 2015-05-13 08:29:07 +0200 (Mi, 13 Mai 2015) $
 * $Revision: 35003 $
 *
 * @remarks
 *
 */
#ifndef __STD_INCLUDES_HEADER
#define __STD_INCLUDES_HEADER

// ADTF header
#include <adtf_platform_inc.h>
#include <adtf_plugin_sdk.h>
#include <additional/adtf_signal_registry_support.h>
//#include <aadc.h>

struct tAADC_Maneuver
{
 int id;
 cString action;
};

struct tSector
{
  int id;
  std::vector<tAADC_Maneuver> maneuverList;
};

using namespace adtf;


#endif // __STD_INCLUDES_HEADER
