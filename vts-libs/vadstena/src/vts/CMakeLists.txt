define_module(BINARY ts2vts
  DEPENDS vadstena-libs)

# ------------------------------------------------------------------------
set(ts2vts_SOURCES
  ts2vts.cpp
  )

add_executable(ts2vts ${ts2vts_SOURCES})
target_link_libraries(ts2vts ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(ts2vts ${MODULE_DEFINITIONS})
buildsys_binary(ts2vts)
set_target_version(ts2vts ${vadstena_VERSION})

# ------------------------------------------------------------------------
# --- installation
# ------------------------------------------------------------------------

# binaries
install(TARGETS ts2vts RUNTIME DESTINATION bin
  COMPONENT storage)
