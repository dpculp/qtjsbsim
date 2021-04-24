#-------------------------------------------------
#
# Project created by QtCreator 2013-09-24T17:03:18
#
#-------------------------------------------------

message("Qt Version "$$QT_VERSION)

QT += widgets
QT += svg network webkitwidgets

TARGET = qtjsbsim
TEMPLATE = app

#-------------------------------------------------

win32: DEFINES += WIN32 _WINDOWS _USE_MATH_DEFINES

win32:CONFIG(release, debug|release):    DEFINES += NDEBUG
else:win32:CONFIG(debug, debug|release): DEFINES += _DEBUG

#-------------------------------------------------

INCLUDEPATH += ./ ./QFI ./autopilot ./debug ./jsbsim ./nav ./viewer \
               ./input ./flightgear \
                /usr/local/include/JSBSim /usr/include/plib

#-------------------------------------------------

LIBS += -L/usr/lib -lplibjs -lplibul

#-------------------------------------------------

HEADERS += \
    MainWindow.h \
    QFI/LayoutSquare.h \
    QFI/qfi_ADI.h \
    QFI/qfi_PFD.h \
    QFI/WidgetPFD.h \
    autopilot/autopilot.h \
    autopilot/pid.h \
    debug/debugtable.h \
    jsbsim/jsbsim.h \
    nav/navigation.h \
    viewer/hud.h \
    viewer/viewer.h \
    input/joystick.h \
    flightgear/packet.h \
    flightgear/net_fdm.hxx \
    flightgear/outsocket.h

SOURCES += \
    main.cpp \
    MainWindow.cpp \
    autopilot/autopilot.cpp \
    autopilot/pid.cpp \
    debug/debugtable.cpp \
    jsbsim/jsbsim.cpp \
    nav/navigation.cpp \
    QFI/LayoutSquare.cpp \
    QFI/qfi_PFD.cpp \
    QFI/WidgetPFD.cpp \
    viewer/hud.cpp \
    viewer/viewer.cpp \
    input/joystick.cpp \
    flightgear/packet.cpp \
    flightgear/outsocket.cpp

FORMS += \
    MainWindow.ui \
    QFI/WidgetPFD.ui \
    debug/debugtable.ui \
    viewer/viewer.ui \

RESOURCES += \
    qtjsbsim.qrc \
    icons.qrc \
    hud.qrc

OTHER_FILES += \
    qtjsbsim.ini

SUBDIRS += \
    qtjsbsim.pro

DISTFILES += \
    images/hud/horizon.png \
    images/hud/hud_base.png \
    images/dkgreen.png \
    images/green.png \
    images/pfd/pfd.svg \
    images/pfd/pfd_adi_back.svg \
    images/pfd/pfd_adi_barh.svg \
    images/pfd/pfd_adi_barv.svg \
    images/pfd/pfd_adi_doth.svg \
    images/pfd/pfd_adi_dotv.svg \
    images/pfd/pfd_adi_ladd.svg \
    images/pfd/pfd_adi_mark.svg \
    images/pfd/pfd_adi_mask.svg \
    images/pfd/pfd_adi_path.svg \
    images/pfd/pfd_adi_roll.svg \
    images/pfd/pfd_adi_slip.svg \
    images/pfd/pfd_adi_turn.svg \
    images/pfd/pfd_alt_back.svg \
    images/pfd/pfd_alt_frame.svg \
    images/pfd/pfd_alt_ground.svg \
    images/pfd/pfd_alt_scale.svg \
    images/pfd/pfd_asi_back.svg \
    images/pfd/pfd_asi_frame.svg \
    images/pfd/pfd_asi_scale.svg \
    images/pfd/pfd_back.svg \
    images/pfd/pfd_hsi_back.svg \
    images/pfd/pfd_hsi_face.svg \
    images/pfd/pfd_hsi_marks.svg \
    images/pfd/pfd_mask.svg \
    images/pfd/pfd_vsi_arrow.svg \
    images/pfd/pfd_vsi_scale.svg \
    qtjsbsim.ini \
    QFI/CMakeLists.txt \
    images/pause.png \
    images/stop.png \
    images/help.png
