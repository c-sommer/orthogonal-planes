CMAKE_MINIMUM_REQUIRED(VERSION 3.4)

SET(EXECUTABLE_OUTPUT_PATH ../../ply_corners/bin)

## ----- corner detection and refinement -----

SET(APP_NAME PLY_Corners)

ADD_EXECUTABLE( ${APP_NAME}
        src/main.cpp
        )

TARGET_LINK_LIBRARIES( ${APP_NAME}
        tinyply
        vis_lib # links to Pangolin_LIBRARIES
        graph_lib # links to CERES_LIBRARIES
        corner_lib
        ppf_lib
        )
        
TARGET_COMPILE_OPTIONS( ${APP_NAME} PRIVATE -std=c++11 )
