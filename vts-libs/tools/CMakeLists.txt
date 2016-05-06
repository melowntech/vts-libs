# VTS tools

if (TARGET service)
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

    add_executable(vts-nodemask EXCLUDE_FROM_ALL nodemask.cpp)
    target_link_libraries(vts-nodemask ${MODULE_LIBRARIES})
    buildsys_target_compile_definitions(vts-nodemask PRIVATE ${MODULE_DEFINITIONS})
    buildsys_binary(vts-nodemask)

    add_executable(vts-measure-dataset EXCLUDE_FROM_ALL measure-dataset.cpp)
    target_link_libraries(vts-measure-dataset ${MODULE_LIBRARIES})
    buildsys_target_compile_definitions(vts-measure-dataset
      PRIVATE ${MODULE_DEFINITIONS})
    buildsys_binary(vts-measure-dataset)

    set(vts_SOURCES vts.cpp)
    if (vts-libs_vts-install_component)
      add_executable(vts ${vts_SOURCES})
      # and install it to along other storage tools
      install(TARGETS vts RUNTIME DESTINATION bin
        COMPONENT ${vts-libs_vts-install_component})
    else()
      add_executable(vts EXCLUDE_FROM_ALL ${vts_SOURCES})
    endif()

    target_link_libraries(vts ${MODULE_LIBRARIES})
    buildsys_target_compile_definitions(vts PRIVATE ${MODULE_DEFINITIONS})
    buildsys_binary(vts)
  endif()
endif()
