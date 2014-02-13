if (NOT DEFINED BUILDSYS_NOBUILD_TARGET_vadstena-libs)
  set(DEPENDS )

  file_to_cpp(PUBLIC_KEY_SOURCES vadstena::matrixSource detail/matrix-source)
  file_to_cpp(PUBLIC_KEY_SOURCES vadstena::fingerPrintManipulator
    detail/fingeprint-manipulator)

  define_module(LIBRARY vadstena-libs DEPENDS
    service cvision optics math utility dbglog jsoncpp
    Boost_FILESYSTEM Boost_REGEX
    OpenCV LIBSHP CRYPTO++ MYSQL++)

  if (BUILDSYS_CUSTOMER_BUILD)
    message(STATUS "Building vadstena-libs customized for customer "
      "'${BUILDSYS_CUSTOMER}'.")
    set(CUSTOMER_SOURCES customer/${BUILDSYS_CUSTOMER}.cpp)
  else()
    message(STATUS "Building vadstena-libs without any customization.")
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

    # crypto stuff
    clog.hpp clog.cpp
    deferredlogger.hpp deferredlogger.cpp
    encryptor.hpp encryptor.cpp

    cameralist.hpp cameralist.cpp
    facetexcoords.hpp facetexcoords.cpp
    gil2cvmat.hpp
    gcp.hpp gcp.cpp
    metatile.hpp metatile.cpp
    binmesh.hpp binmesh.cpp

    shapefile.hpp shapefile.cpp

    viewconfig.hpp viewconfig.cpp
    flowconfig.hpp flowconfig.cpp
    fitting.hpp fitting.cpp
    fittingset.hpp fittingset.cpp
    features.hpp features.cpp

    tilestorage.hpp
    tilestorage/tileop.hpp
    tilestorage/error.hpp
    tilestorage/storage.hpp tilestorage/storage.cpp
    tilestorage/tileset.hpp tilestorage/tileset.cpp
    tilestorage/io.hpp tilestorage/po.hpp
    tilestorage/json.hpp tilestorage/json.cpp
    tilestorage/tileindex.hpp tilestorage/tileindex.cpp
    tilestorage/driver.hpp tilestorage/driver/driver.cpp
    tilestorage/driver/flat.hpp tilestorage/driver/flat.cpp
    tilestorage/merge.hpp tilestorage/merge.cpp

    licence/fingerprint.hpp licence/fingerprint.cpp

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
