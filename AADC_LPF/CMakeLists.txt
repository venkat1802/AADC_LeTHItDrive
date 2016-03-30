# add the include directory of the filter
include_directories(../include)

# declare our filter
adtf_add_filter(aadc_lpf            
    stdafx.h            
    stdafx.cpp            
    lpf.h            
    lpf.cpp
)

adtf_install_plugin(aadc_lpf ${AADC_INSTALL_DIR})
