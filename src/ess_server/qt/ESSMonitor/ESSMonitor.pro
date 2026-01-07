QT       += core gui charts sql network

LIBS += -lmosquitto

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    batteryrackwidget.cpp \
    essmapwidget.cpp \
    loginwindow.cpp \
    main.cpp \
    mainwindow.cpp \
    tab1main.cpp \
    tab2env.cpp \
    tab3alert.cpp \
    tab4access.cpp

HEADERS += \
    batteryrackwidget.h \
    essmapwidget.h \
    loginwindow.h \
    mainwindow.h \
    tab1main.h \
    tab2env.h \
    tab3alert.h \
    tab4access.h

FORMS += \
    batteryrackwidget.ui \
    essmapwidget.ui \
    loginwindow.ui \
    mainwindow.ui \
    tab1main.ui \
    tab2env.ui \
    tab3alert.ui \
    tab4access.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

RESOURCES += \
    resources.qrc
