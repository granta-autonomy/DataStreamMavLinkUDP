# MAVlink plugin for PlotJuggler
Currently it supports only `mavlink_local_position_ned_t` and `mavlink_local_position_ned_cov_t` messages for visualization, but other messages can be added too.

## Build
- Checkout `https://github.com/facontidavide/PlotJuggler` repository first
- Then,inside of `plotjuggler_plugins` checkout `https://github.com/granta-autonomy/DataStreamMavLinkUDP.git`
- Add the following patch to `PlotJuggler/CMakeLists.txt` file
```patch
@@ -266,9 +276,10 @@ add_subdirectory( plotjuggler_plugins/DataLoadULog )

 add_subdirectory( plotjuggler_plugins/DataStreamSample )
 add_subdirectory( plotjuggler_plugins/DataStreamWebsocket )
+add_subdirectory( plotjuggler_plugins/DataStreamMavLinkUDP )
 add_subdirectory( plotjuggler_plugins/DataStreamUDP )
 add_subdirectory( plotjuggler_plugins/DataStreamMQTT )
 ```
 - Compile PlotJuggler