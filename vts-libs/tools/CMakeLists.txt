# VTS tools

define_module(BINARY vts-tools
  DEPENDS vts-libs service
)

add_executable(vts-nodeinfo EXCLUDE_FROM_ALL nodeinfo.cpp)
target_link_libraries(vts-nodeinfo ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(vts-nodeinfo PRIVATE ${MODULE_DEFINITIONS})
buildsys_binary(vts-nodeinfo)

add_executable(vts-nodemask EXCLUDE_FROM_ALL nodemask.cpp)
target_link_libraries(vts-nodemask ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(vts-nodemask PRIVATE ${MODULE_DEFINITIONS})
buildsys_binary(vts-nodemask)
