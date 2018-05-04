
ENABLE_LANGUAGE(CXX)

SET(SRC_OBJS
     jsbsim/jsbsim.cpp
     main.cpp
     MainWindow.cpp
     viewer/viewer.cpp
     viewer/hud.cpp
     debug/debugtable.cpp
     nav/navigation.cpp
     autopilot/pid.cpp
     autopilot/autopilot.cpp
     input/joystick.cpp
     QFI/LayoutSquare.cpp
     QFI/qfi_PFD.cpp
     QFI/WidgetPFD.cpp
     flightgear/packet.cpp
     flightgear/outsocket.cpp
   )

SET(SRC_HDRS
     jsbsim/jsbsim.h
     MainWindow.h
     viewer/viewer.h
     viewer/hud.h
     debug/debugtable.h
     nav/navigation.h
     autopilot/pid.h
     autopilot/autopilot.h
     input/joystick.h
     QFI/LayoutSquare.h
     QFI/qfi_ADI.h
     QFI/qfi_PFD.h
     QFI/WidgetPFD.h
     flightgear/net_fdm.hxx
     flightgear/packet.h
     flightgear/outsocket.h
   )

SET(GUI_FILES
     MainWindow.ui
     viewer/viewer.ui
     debug/debugtable.ui
     QFI/WidgetPFD.ui
   )

SET(RCS_FILES
     qtjsbsim.qrc
     icons.qrc
     hud.qrc
   )

QT5_WRAP_UI(UI_SRC ${GUI_FILES})
QT5_WRAP_CPP(MOC_SRC ${SRC_HDRS})
QT5_ADD_RESOURCES(RCC_SRC ${RCS_FILES})

ADD_EXECUTABLE( qtjsbsim ${SRC_OBJS} ${MOC_SRC} ${UI_SRC} ${RCC_SRC} )
TARGET_LINK_LIBRARIES( qtjsbsim ${PLIB_LIBRARIES} )
QT5_USE_MODULES(qtjsbsim Gui Network Svg WebKit WebKitWidgets)
