# VTS tools

if (TARGET service)
  message(STATUS "vts-libs: building tools")
  if(TARGET vts-libs)
    define_module(BINARY vts-core-tools
      DEPENDS vts-libs-core service
      )

    add_executable(vts-nodeinfo EXCLUDE_FROM_ALL nodeinfo.cpp)
    target_link_libraries(vts-nodeinfo ${MODULE_LIBRARIES})
    target_compile_definitions(vts-nodeinfo PRIVATE ${MODULE_DEFINITIONS})
    buildsys_binary(vts-nodeinfo)
  endif()

  if(TARGET vts-libs)
    set(EXTRA_DEPENDS)
    if(MAGIC_FOUND)
      list(APPEND EXTRA_DEPENDS MAGIC)
    endif()

    if (TARGET vts-libs-tools-support)
      list(APPEND EXTRA_DEPENDS vts-libs-tools-support)
    endif()

    ###################################################
    # tools
    define_module(BINARY vts-tools
      DEPENDS vts-libs service ${EXTRA_DEPENDS}
      )

    add_executable(vts-nodemask EXCLUDE_FROM_ALL nodemask.cpp)
    target_link_libraries(vts-nodemask ${MODULE_LIBRARIES})
    target_compile_definitions(vts-nodemask PRIVATE ${MODULE_DEFINITIONS})
    buildsys_binary(vts-nodemask)

    add_executable(vts-measure-dataset EXCLUDE_FROM_ALL measure-dataset.cpp)
    target_link_libraries(vts-measure-dataset ${MODULE_LIBRARIES})
    target_compile_definitions(vts-measure-dataset
      PRIVATE ${MODULE_DEFINITIONS})
    buildsys_binary(vts-measure-dataset)

    macro(vts_libs_tool name)
      message(STATUS "vts-libs: building ${name} tool")
      add_executable(${name} ${ARGN})
      if (vts-libs_vts-install_component)
        install(TARGETS ${name} RUNTIME DESTINATION bin
          COMPONENT ${vts-libs_vts-install_component})
      endif()

      target_link_libraries(${name} ${MODULE_LIBRARIES})
      target_compile_definitions(${name} PRIVATE ${MODULE_DEFINITIONS})
      buildsys_binary(${name})
    endmacro()

    vts_libs_tool(vts vts.cpp locker.hpp locker.cpp support/urlfetcher.cpp)
    if (TARGET vts-libs-tools-support)
      target_compile_definitions(vts PRIVATE VTS_LIBS_HAS_TOOLS_SUPPORT=1)
    endif()

    vts_libs_tool(vts2vts vts2vts.cpp)
    vts_libs_tool(vts2dem vts2dem.cpp)
    vts_libs_tool(vts2ophoto vts2ophoto.cpp)

    vts_libs_tool(tilar tilar.cpp)

    vts_libs_tool(mapconfig mapconfig.cpp support/urlfetcher.cpp)

    vts_libs_tool(urltemplate urltemplate.cpp)

    vts_libs_tool(vtsrepack repack.cpp)
  endif()
endif()
