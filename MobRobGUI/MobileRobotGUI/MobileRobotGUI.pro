#-------------------------------------------------
#
# Project created by QtCreator 2015-11-25T18:11:23
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = MobileRobotGUI
TEMPLATE = app


SOURCES += main.cpp\
        gui.cpp \
    visualization.cpp

HEADERS  += gui.h \
    visualization.h

FORMS    += gui.ui



INCLUDEPATH += /usr/local/include
LIBS += `pkg-config --libs opencv`
