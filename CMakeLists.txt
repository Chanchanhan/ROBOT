cmake_minimum_required(VERSION 3.1)
project(ROBOT)
enable_testing()
set( CMAKE_CXX_COMPILER "g++" )

# set(CMAKE_CXX_STANDARD 11)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/CmakeModules/)

##############################################################################
# C++14 support
##############################################################################
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX14)
  set( CMAKE_CXX_FLAGS "-std=c++14 -march=native " )
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support. Some functionality will not be available.")
endif()


find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(glog  REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

file(GLOB_RECURSE SOURCE_FILES  ${PROJECT_SOURCE_DIR}/src/*.c* ${PROJECT_SOURCE_DIR}/ThirdParty/*.c*)
file(GLOB_RECURSE HEADERS  ${PROJECT_SOURCE_DIR}/include/*.h* ${PROJECT_SOURCE_DIR}/ThirdParty/*.h*)

source_group("Include" FILES ${HEADERS}) 
source_group("Src" FILES ${SOURCE_FILES}) 


INCLUDE_DIRECTORIES(
  ${OpenCV_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include/
  ${OPENGL_INCLUDE_DIRS}
  ${GLUT_INCLUDE_DIRS}
  ${GLOG_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)


ADD_DEFINITIONS(/D_USE_MATH_DEFINES)

#LIST(APPEND LINK_LIBS ${PROJECT_SOURCE_DIR}/DBoW3001d.lib ${OpenCV_LIBS})
LIST(APPEND LINK_LIBS ${GLOG_LIBRARY}  ${OpenCV_LIBS} ${Sophus_LIBRARIES}  ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} )
#TARGET_LINK_LIBRARIES(ROBOT ${LINK_LIBS})



#add test unit 1
add_executable(test-cgy TestUnits/main.cpp ${HEADERS} ${SOURCE_FILES} )
add_test(test-cgy test-cgy)
SET_TARGET_PROPERTIES(test-cgy PROPERTIES    RUNTIME_OUTPUT_DIRECTORY  "${PROJECT_SOURCE_DIR}/bin")
TARGET_LINK_LIBRARIES(test-cgy ${LINK_LIBS})


#add test unit 2
add_executable(test-DataProcessor TestUnits/test_DataProcessor.cpp  ${HEADERS} ${SOURCE_FILES})
add_test(test-DataProcessor test-DataProcessor)
SET_TARGET_PROPERTIES(test-DataProcessor PROPERTIES    RUNTIME_OUTPUT_DIRECTORY  "${PROJECT_SOURCE_DIR}/bin")
TARGET_LINK_LIBRARIES( test-DataProcessor  ${LINK_LIBS} ) 