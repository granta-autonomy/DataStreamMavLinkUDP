
include_directories( ../ )

add_definitions(${QT_DEFINITIONS})
add_definitions(-DQT_PLUGIN)

QT5_WRAP_UI ( UI_SRC  udp_mavlink_server.ui  )

SET( SRC udp_mavlink_server.cpp )

add_library(DataStreamMavLinkUDP SHARED ${SRC} ${UI_SRC}  )

target_link_libraries(DataStreamMavLinkUDP
    ${Qt5Widgets_LIBRARIES}
    Qt5::Network
    plotjuggler_base
    )

install(TARGETS DataStreamMavLinkUDP DESTINATION ${PJ_PLUGIN_INSTALL_DIRECTORY}  )
