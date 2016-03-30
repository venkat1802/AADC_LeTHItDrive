# add the include directory of the filter
include_directories(../include)

# declare our filter
adtf_add_filter(aadc_freedemo            
    stdafx.h            
    stdafx.cpp            
    freedemo.h            
    freedemo.cpp
)

adtf_install_plugin(aadc_freedemo ${AADC_INSTALL_DIR})
