cmake_minimum_required(VERSION 3.5)

project(pathlib LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# QtCreator supports the following variables for Android, which are identical to qmake Android variables.
# Check http://doc.qt.io/qt-5/deployment-android.html for more information.
# They need to be set before the find_package(Qt5 ...) call.

#if(ANDROID)
#    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
#    if (ANDROID_ABI STREQUAL "armeabi-v7a")
#        set(ANDROID_EXTRA_LIBS
#            ${CMAKE_CURRENT_SOURCE_DIR}/path/to/libcrypto.so
#            ${CMAKE_CURRENT_SOURCE_DIR}/path/to/libssl.so)
#    endif()
#endif()

set(SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
set(BUILD_DIR "${CMAKE_CURRENT_BINARY_DIR}")

# Set path to eigen3, for install $ sudo apt-get install libeigen3-dev
include_directories("/usr/include/eigen3/Eigen")
include_directories(${SRC_DIR}/oce-upstream-V7_5_0beta/inc)
include_directories(${SRC_DIR}/oce-upstream-V7_5_0beta/src)
include_directories(${SRC_DIR}/qt_occ_draw)

find_package(QT NAMES Qt6 Qt5 COMPONENTS Widgets REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets REQUIRED)


if(ANDROID)
  add_library(pathlib SHARED

  )
else()
  add_executable(pathlib
    main.cpp
    mainwindow.cpp
    mainwindow.h
    mainwindow.ui
    pathlib.h
    pathlib.cpp
    pathvector.h
    pathvector.cpp
    PointOnArc.h
    ${SRC_DIR}/qt_occ_draw/OcctQtViewer.h
    ${SRC_DIR}/qt_occ_draw/OcctQtViewer.cpp
    ${SRC_DIR}/qt_occ_draw/draw_primitives.h
    ${SRC_DIR}/qt_occ_draw/draw_primitives.cpp
    ${SRC_DIR}/ruckig/ruckig_format.h
    ${SRC_DIR}/ruckig/ruckig_interface.h
    ${SRC_DIR}/ruckig/ruckig_interface.cpp
    ${SRC_DIR}/ruckig/brake.hpp
    ${SRC_DIR}/ruckig/brake.cpp
    ${SRC_DIR}/ruckig/position-step1.cpp
    ${SRC_DIR}/ruckig/position-step2.cpp
    ${SRC_DIR}/ruckig/velocity-step1.cpp
    ${SRC_DIR}/ruckig/velocity-step2.cpp
  )
endif()

# Opencascade
find_package (OpenCASCADE REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

SET(OpenCASCADE_LIBS
    TKGeomAlgo TKMesh TKHLR TKBO TKShHealing
    TKPrim
    TKernel TKMath TKTopAlgo TKService
    TKG2d TKG3d TKV3d TKOpenGl
    TKBRep TKXSBase TKGeomBase TKGeomAlgo
    TKXSDRAW
    TKLCAF TKXCAF TKCAF TKVCAF
    TKCDF TKBin TKBinL TKBinXCAF TKXml TKXmlL TKXmlXCAF
    # -- IGES support
    TKIGES
    # -- STEP support
    TKSTEP TKXDESTEP TKXDEIGES TKSTEPAttr TKSTEPBase TKSTEP209
    # -- STL support
    TKSTL
    # -- OBJ/glTF support
    TKRWMesh TKMeshVS
    # -- VRML support
    TKVRML
    # -- ViewerTest
    TKViewerTest
    )

target_link_libraries(pathlib PRIVATE Qt${QT_VERSION_MAJOR}::Widgets ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${OpenCASCADE_LIBS})
