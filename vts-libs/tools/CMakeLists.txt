# VTS tools

define_module(BINARY vts-tools
  DEPENDS vts-libs-core service
)

add_executable(vts-nodeinfo EXCLUDE_FROM_ALL nodeinfo.cpp)
target_link_libraries(vts-nodeinfo ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(vts-nodeinfo PRIVATE ${MODULE_DEFINITIONS})
buildsys_binary(vts-nodeinfo)
