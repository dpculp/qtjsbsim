
ENABLE_LANGUAGE(CXX)

#Add qt-gui files
FILE(GLOB qt-gui_srcs *.cpp)
FILE(GLOB qt-gui_hdrs *.h)

# Generate QT Files
FILE(GLOB Forms_UIS *.ui)
FILE(GLOB RCC_FILES *.qrc)

QT5_WRAP_UI(UI_SRC ${Forms_UIS})
QT5_WRAP_CPP(MOC_SRC ${qt-gui_hdrs})
QT5_ADD_RESOURCES(RCC_SRC ${RCC_FILES})

ADD_EXECUTABLE( qtjsbsim ${qt-gui_srcs} ${MOC_SRC} ${UI_SRC} ${RCC_SRC} )
TARGET_LINK_LIBRARIES( qtjsbsim ${QT_LIBRARIES} ${PLIB_LIBRARIES} )
