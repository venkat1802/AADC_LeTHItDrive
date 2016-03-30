set(FILTER_NAME PullOut)
 
include_directories(../include)# include directories
include_directories(${OpenCV_INCLUDE_DIR}) 

# Specify the filter module target
adtf_add_filter(${FILTER_NAME}
                stdafx.cpp     
                stdafx.h
                pullout.h
                pullout.cpp
)


# linking libs
target_link_libraries(${FILTER_NAME} ${OpenCV_LIBS})

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
