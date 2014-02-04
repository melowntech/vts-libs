if (NOT DEFINED BUILDSYS_NOBUILD_TARGET_vadstena-libs)
  set(OPTIONAL_DEPENDS)
  set(OPTIONAL_SOURCES)

  if (LIBSHP_FOUND)
    list(APPEND OPTIONAL_DEPENDS LIBSHP Boost_REGEX)
    list(APPEND OPTIONAL_SOURCES
      shapefile.hpp shapefile.cpp)
  endif()

  if (MODULE_cvision_FOUND)
    list(APPEND OPTIONAL_DEPENDS cvision optics)
    list(APPEND OPTIONAL_SOURCES
      viewconfig.hpp viewconfig.cpp
      flowconfig.hpp flowconfig.cpp
      fitting.hpp fitting.cpp
      fittingset.hpp fittingset.cpp
      features.hpp features.cpp
      )
  endif()

  file_to_cpp(PUBLIC_KEY_SOURCES vadstena::matrixSource detail/matrix-source)

  define_module(LIBRARY vadstena-libs DEPENDS ${OPTIONAL_DEPENDS}
    CRYPTO++ MYSQL++ service math utility dbglog jsoncpp Boost_FILESYSTEM)

  if (BUILDSYS_CUSTOMER_MODE)
    set(CUSTOMER_SOURCES customer/${BUILDSYS_CUSTOMER}.cpp)
  else()
    set(CUSTOMER_SOURCES customer/none.cpp)
  endif()

  # add browser files
  file_to_cpp(vadstena-libs_BROWSER_SOURCES
    vadstena::tilestorage::browser::index_html
    tilestorage/browser/index.html)
  file_to_cpp(vadstena-libs_BROWSER_SOURCES
    vadstena::tilestorage::browser::skydome_jpg
    tilestorage/browser/skydome.jpg)

  set(vadstena-libs_SOURCES
    db.hpp db.cpp
    file.hpp
    entities.hpp entities.cpp
    utility.hpp utility.cpp
    tiling.hpp tiling.cpp
    copyfile.hpp copyfile.cpp
    colorconfig.hpp colorconfig.cpp
    entities.hpp
    pc.hpp pc.cpp
    uvpack.hpp uvpack.cpp
    mesh.hpp mesh.cpp
    clog.hpp clog.cpp
    deferredlogger.hpp deferredlogger.cpp
    cameralist.hpp cameralist.cpp
    facetexcoords.hpp facetexcoords.cpp
    gil2cvmat.hpp
    gcp.hpp gcp.cpp
    metatile.hpp metatile.cpp
    binmesh.hpp binmesh.cpp

    tilestorage.hpp tilestorage/tilestorage.cpp
    tilestorage/tileop.hpp
    tilestorage/io.hpp
    tilestorage/json.hpp tilestorage/json.cpp
    tilestorage/fs.hpp tilestorage/fs.cpp

    ${OPTIONAL_SOURCES}

    ${PUBLIC_KEY_SOURCES}

    customer.hpp ${CUSTOMER_SOURCES}

    ${vadstena-libs_BROWSER_SOURCES}
    )

  # fallback to non-C++11 standard for this helper file
  if (${CMAKE_C_COMPILER_ID} MATCHES GNU)
    set_source_files_properties(copyfile.cpp PROPERTIES
      COMPILE_FLAGS -std=c++98)
  endif()

  add_library(vadstena-libs STATIC ${vadstena-libs_SOURCES})
  set_target_properties(vadstena-libs PROPERTIES ARCHIVE_OUTPUT_DIRECTORY
    ${CMAKE_SOURCE_DIR}/lib)

  target_link_libraries(vadstena-libs ${MODULE_LIBRARIES})
endif()

# private library; NOT to be USED in customer code!
if (DEFINED BUILDSYS_BUILD_TARGET_vadstena-libs-private)
  define_module(LIBRARY vadstena-libs-private DEPENDS CRYPTO++)

  file_to_cpp(PRIVATE_KEY_SOURCES vadstena::matrixSink detail/matrix-sink)

  set(vadstena-libs-private_SOURCES
    private-key.hpp private-key.cpp
    ${PRIVATE_KEY_SOURCES}
    )

  add_library(vadstena-libs-private STATIC ${vadstena-libs-private_SOURCES})
  set_target_properties(vadstena-libs-private PROPERTIES ARCHIVE_OUTPUT_DIRECTORY
    ${CMAKE_SOURCE_DIR}/lib)

  target_link_libraries(vadstena-libs-private ${MODULE_LIBRARIES})
endif()
