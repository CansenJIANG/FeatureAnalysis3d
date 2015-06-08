#-------------------------------------------------
#
# Project created by QtCreator 2014-05-01T14:24:33
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = FeatureAnalysis3d
TEMPLATE = app

CONFIG +=link_pkgconfig
CONFIG += console
#PKGCONFIG = opencv-2.4.10
#PKGCONFIG = libfreenect

SOURCES += src/main.cpp\
           src/pclviewer.cpp \
           src/extractFeatures.cpp

HEADERS  += src/pclviewer.h \
            src/extractFeatures.h \
            src/commonFunc.h \
            src/commonHeader.h

FORMS    += src/PclViewer.ui

INCLUDEPATH += /usr/include/vtk-5.8 \
                /usr/local/include/pcl-1.8 \
                /home/jiang/CvLibs/eigen3.2.3 \
                /usr/include/boost \
                /usr/include/ni

LIBS += -L/usr/local/lib \
            -lpcl_io \
            -lpcl_visualization \
            -lpcl_common \
            -lpcl_filters \
            -lpcl_octree \
            -lpcl_kdtree \
            -lpcl_registration \
            -lpcl_features \
            -lpcl_keypoints \
            -lpcl_recognition \
            -lpcl_sample_consensus \
            -lpcl_search  \
            -lpcl_surface \
            -lpcl_ml \
            -lpcl_outofcore \
            -lpcl_people \
            -lpcl_segmentation \
            -lpcl_tracking \
            -lpcl_stereo

LIBS += -L/usr/lib \
            -lboost_filesystem \
            -lboost_thread \
            -lboost_system \
            -lQVTK \
            -lvtkRendering \
            -lvtkIO \
            -lvtkCommon \
            -lvtkWidgets \
            -lvtkFiltering \
            -lvtkGraphics


