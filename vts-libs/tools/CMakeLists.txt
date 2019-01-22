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
    set(EXTRA_DEPENDS)
    if(MAGIC_FOUND)
      set(EXTRA_DEPENDS MAGIC)
    endif()

    ###################################################
    # tools
    define_module(BINARY vts-tools
      DEPENDS vts-libs service ${EXTRA_DEPENDS}
      )

    macro(vts_libs_tool name)
      message(STATUS "vts-libs: building ${name} tool")
      add_executable(${name} ${ARGN})
      if (vts-libs_vts-install_component)
        install(TARGETS ${name} RUNTIME DESTINATION bin
          COMPONENT ${vts-libs_vts-install_component})
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

    vts_libs_tool(vts vts.cpp locker.hpp locker.cpp support/urlfetcher.cpp)

    vts_libs_tool(vts2vts vts2vts.cpp)
    vts_libs_tool(vts02vts vts02vts.cpp)
    vts_libs_tool(vts2dem vts2dem.cpp)
    vts_libs_tool(vts2ophoto vts2ophoto.cpp)

    vts_libs_tool(tilar tilar.cpp)

    vts_libs_tool(mapconfig mapconfig.cpp support/urlfetcher.cpp)

    vts_libs_tool(urltemplate urltemplate.cpp)

    vts_libs_tool(vts0
      vts0/vts0.cpp vts0/commands.hpp vts0/fallback.cpp
      vts0/create.cpp vts0/info.cpp vts0/add.cpp
      vts0/tileset-info.cpp vts0/tileset-clone.cpp
      vts0/tileset-paste.cpp)
  endif()
endif()
