set(FILTER_NAME aadc_speedcontroller)
 
# Specify the filter module target
adtf_add_filter(${FILTER_NAME}
                stdafx.cpp     
                stdafx.h
                cspeedcontroller.h
                cspeedcontroller.cpp
)

# set folder (only needed and available for MSVC)
adtf_set_folder(${FILTER_NAME} demo) 

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
