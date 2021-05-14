set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
if (NOT yaml-cpp_FOUND)
find_package (yaml-cpp REQUIRED)
endif()
if (NOT PCL_FOUND)
    find_package(PCL REQUIRED)
endif ()
if (NOT OPENCV_FOUND)
    find_package(OpenCV  REQUIRED)
endif ()
file(GLOB SRC_LIST *.cpp)
add_library(ssc SHARED ${SRC_LIST})
target_include_directories(ssc PUBLIC 
${PCL_INCLUDE_DIRS} 
${YAML_CPP_INCLUDE_DIR} 
./
)
target_link_libraries(ssc
        ${PCL_LIBRARIES}
	${YAML_CPP_LIBRARIES}
yaml-cpp
        ${OpenCV_LIBS})
