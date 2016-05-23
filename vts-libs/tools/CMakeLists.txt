# VTS tools

if (TARGET service)
  message(STATUS "vts-libs: building tools")
  if(TARGET vts-libs)
    define_module(BINARY vts-core-tools
      DEPENDS vts-libs-core service
      )

    add_executable(vts-nodeinfo EXCLUDE_FROM_ALL nodeinfo.cpp)
    target_link_libraries(vts-nodeinfo ${MODULE_LIBRARIES})
    buildsys_target_compile_definitions(vts-nodeinfo PRIVATE ${MODULE_DEFINITIONS})
    buildsys_binary(vts-nodeinfo)
  endif()

  if(TARGET vts-libs)
    define_module(BINARY vts-tools
      DEPENDS vts-libs service
      )

    macro(vts_libs_tool name)
      if (vts-libs_vts-install_component)
        add_executable(${name} ${ARGN})
        # and install it to along other storage tools
        install(TARGETS ${name} RUNTIME DESTINATION bin
          COMPONENT ${vts-libs_vts-install_component})
      else()
        add_executable(${name} EXCLUDE_FROM_ALL ${ARGN})
      endif()

      target_link_libraries(${name} ${MODULE_LIBRARIES})
      buildsys_target_compile_definitions(${name} PRIVATE ${MODULE_DEFINITIONS})
      buildsys_binary(${name})
    endmacro()

    add_executable(vts-nodemask EXCLUDE_FROM_ALL nodemask.cpp)
    target_link_libraries(vts-nodemask ${MODULE_LIBRARIES})
    buildsys_target_compile_definitions(vts-nodemask PRIVATE ${MODULE_DEFINITIONS})
    buildsys_binary(vts-nodemask)

    add_executable(vts-measure-dataset EXCLUDE_FROM_ALL measure-dataset.cpp)
    target_link_libraries(vts-measure-dataset ${MODULE_LIBRARIES})
    buildsys_target_compile_definitions(vts-measure-dataset
      PRIVATE ${MODULE_DEFINITIONS})
    buildsys_binary(vts-measure-dataset)

    vts_libs_tool(vts vts.cpp)
    vts_libs_tool(vts2vts vts2vts.cpp heightmap.hpp heightmap.cpp)
    vts_libs_tool(vts02vts vts02vts.cpp heightmap.hpp heightmap.cpp)
  endif()
endif()
