set(FILTER_NAME initparkparallel)

# add the include directory of the filter
include_directories(../include)# include directories
include_directories(${OpenCV_INCLUDE_DIR}) 

# add the source
adtf_add_filter(${FILTER_NAME}
                stdafx.cpp
                stdafx.h
                #PPinit.h
                #PPinit1.cpp
		parallelparkstart.h
		parallelparkstart.cpp
 )

# linking libs
target_link_libraries(${FILTER_NAME} ${OpenCV_LIBS})

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
