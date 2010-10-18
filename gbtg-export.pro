# -------------------------------------------------
# Project created by QtCreator 2010-10-14T12:09:52
# -------------------------------------------------
QT -= core \
    gui
TARGET = gbtg-export
CONFIG += console
CONFIG -= app_bundle
TEMPLATE = app
SOURCES += gbtg-export.cpp
LIBS += -lboost_date_time -lboost_filesystem -lboost_program_options
