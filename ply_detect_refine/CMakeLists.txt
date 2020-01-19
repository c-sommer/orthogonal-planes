CMAKE_MINIMUM_REQUIRED(VERSION 3.4)

SET(EXECUTABLE_OUTPUT_PATH ../../ply_detect_refine/bin)

## ----- plane pair detection -----

SET(APP_NAME PLY_PPDetectRefine)

ADD_EXECUTABLE( ${APP_NAME}
        src/main.cpp
        )

TARGET_LINK_LIBRARIES( ${APP_NAME}
        tinyply
        vis_lib # links to Pangolin_LIBRARIES
        graph_lib # links to CERES_LIBRARIES
        ppf_lib
        )
        
TARGET_COMPILE_OPTIONS( ${APP_NAME} PRIVATE -std=c++11 )
