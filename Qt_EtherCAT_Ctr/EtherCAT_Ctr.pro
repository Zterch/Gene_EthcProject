QT       += core gui network

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11

TARGET = EtherCAT_Ctr

SOURCES += \
    main.cpp \
    mainwindow.cpp \
    core/ethercatcommander.cpp \
    core/udpclient.cpp \
    widgets/multiaxiscontroldialog.cpp \
    widgets/pidconfigtab.cpp \
    widgets/statuspanel.cpp \
    utils/settings.cpp \
    utils/stylehelper.cpp

HEADERS += \
    mainwindow.h \
    core/ethercatcommander.h \
    core/udpclient.h \
    widgets/multiaxiscontroldialog.h \
    widgets/pidconfigtab.h \
    widgets/statuspanel.h \
    utils/settings.h \
    utils/stylehelper.h

FORMS += \
    mainwindow.ui \
    widgets/multiaxiscontroldialog.ui \
    widgets/pidconfigtab.ui \
    widgets/statuspanel.ui

RESOURCES += \
    resources.qrc

# 包含路径
INCLUDEPATH += . \
               core \
               widgets \
               utils

# 构建目录
DESTDIR = $$PWD/build
OBJECTS_DIR = $$DESTDIR/.obj
MOC_DIR = $$DESTDIR/.moc
UI_DIR = $$DESTDIR/.ui
RCC_DIR = $$DESTDIR/.rcc

# 禁用警告
QMAKE_CXXFLAGS += -Wno-unused-parameter

# 目标路径
target.path = /usr/local/bin
INSTALLS += target
