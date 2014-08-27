if (NOT DEFINED BUILDSYS_NOBUILD_TARGET_vadstena-libs)
  file_to_cpp(PUBLIC_KEY_SOURCES vadstena::matrixSource detail/matrix-source)
  file_to_cpp(PUBLIC_KEY_SOURCES vadstena::fingerPrintManipulator
    detail/fingeprint-manipulator)

  define_module(LIBRARY vadstena-libs DEPENDS
    service cvision optics math geo utility dbglog jsoncpp
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
    vadstena::tilestorage::browser::index_offline_html
    tilestorage/browser/index-offline.html)
  file_to_cpp(vadstena-libs_BROWSER_SOURCES
    vadstena::tilestorage::browser::skydome_jpg
    tilestorage/browser/skydome.jpg)

  set(vadstena-libs_SOURCES
    db.hpp db.cpp
    file.hpp
    entities.hpp entities.cpp

    utility.hpp
    utility/utility.cpp
    utility/job.cpp
    utility/maintenance.cpp
    utility/convert.hpp utility/convert.cpp
    utility/detail.hpp

    tiling.hpp tiling.cpp
    copyfile.hpp copyfile.cpp
    colorconfig.hpp colorconfig.cpp
    entities.hpp
    pc.hpp pc.cpp
    uvpack.hpp uvpack.cpp
    mesh.hpp mesh.cpp
    faceclip.hpp faceclip.cpp
    binmesh.hpp binmesh.cpp

    memory-requirements.hpp

    # crypto stuff
    clog.hpp clog.cpp
    deferredlogger.hpp deferredlogger.cpp
    encryptor.hpp encryptor.cpp

    cameralist.hpp cameralist.cpp
    facetexcoords.hpp facetexcoords.cpp
    gil2cvmat.hpp
    gcp.hpp gcp.cpp

    rectalloc.hpp rectalloc.cpp
    viewrasterize.hpp viewrasterize.cpp
    viewquality.hpp viewquality.cpp
    orthopreview.hpp orthopreview.cpp

    coverage-map.hpp coverage-map.cpp

    shapefile.hpp shapefile.cpp

    viewconfig.hpp viewconfig.cpp
    flowconfig.hpp flowconfig.cpp
    fitting.hpp fitting.cpp
    fittingset.hpp fittingset.cpp
    features.hpp features.cpp
    filelist.hpp filelist.cpp

    measurements.hpp measurements.cpp

    tilestorage.hpp
    tilestorage/basetypes.hpp tilestorage/types.hpp
    tilestorage/properties.hpp
    tilestorage/tileop.hpp
    tilestorage/error.hpp
    tilestorage/storage.hpp tilestorage/storage.cpp
    tilestorage/tileset.hpp tilestorage/tileset.cpp
    tilestorage/io.hpp tilestorage/po.hpp
    tilestorage/json.hpp tilestorage/json.cpp
    tilestorage/tileindex.hpp tilestorage/tileindex.cpp
    tilestorage/metatile.hpp tilestorage/metatile.cpp
    tilestorage/driver.hpp tilestorage/driver/driver.cpp
    tilestorage/driver/fsbased.hpp tilestorage/driver/fsbased.cpp
    tilestorage/driver/flat.hpp
    tilestorage/driver/hash-crc.hpp tilestorage/driver/hash-crc.cpp
    tilestorage/merge.hpp tilestorage/merge.cpp
    tilestorage/tileset/merge.cpp
    tilestorage/tileset/dump.hpp

    licence/fingerprint.hpp licence/fingerprint.cpp

    ${PUBLIC_KEY_SOURCES}

    customer.hpp ${CUSTOMER_SOURCES}

    ${vadstena-libs_BROWSER_SOURCES}
    )

  if (CGAL_FOUND)
    list(APPEND vadstena-libs_SOURCES
      orthopreview.hpp orthopreview.cpp
    )
  endif()

  # fallback to non-C++11 standard for this helper file
  if (${CMAKE_C_COMPILER_ID} MATCHES GNU)
    set_source_files_properties(copyfile.cpp PROPERTIES
      COMPILE_FLAGS -std=c++98)
  endif()

  add_library(vadstena-libs STATIC ${vadstena-libs_SOURCES})
  buildsys_library(vadstena-libs)
  target_link_libraries(vadstena-libs ${MODULE_LIBRARIES})
  buildsys_target_compile_definitions(vadstena-libs ${MODULE_DEFINITIONS})
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
  buildsys_library(vadstena-libs-private)
  target_link_libraries(vadstena-libs-private ${MODULE_LIBRARIES})
  buildsys_target_compile_definitions(vadstena-libs-private ${MODULE_DEFINITIONS})
endif()
