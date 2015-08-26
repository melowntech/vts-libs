# common storage stuff
set(vadstena-libs-storage_SOURCES
  storage/error.hpp
  storage/openfiles.hpp storage/openfiles.cpp
  storage/filetypes.hpp
  storage/streams.hpp storage/streams.cpp
  storage/fstreams.hpp storage/fstreams.cpp

  storage/tilar.hpp storage/tilar.cpp
  storage/tilar-io.hpp

  storage/support.hpp
  )

define_module(LIBRARY vadstena-libs-storage DEPENDS
  math utility dbglog half jsoncpp Boost_IOSTREAMS)
add_library(vadstena-libs-storage STATIC ${vadstena-libs-storage_SOURCES})
buildsys_library(vadstena-libs-storage)
target_link_libraries(vadstena-libs-storage
  ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(vadstena-libs-storage ${MODULE_DEFINITIONS})

# Create vadstena-tileset-browser library with browser files
file_to_cpp(vadstena-libs_BROWSER_SOURCES
  vadstena::tilestorage::browser::index_html
  tilestorage/browser/index.html)
file_to_cpp(vadstena-libs_BROWSER_SOURCES
  vadstena::tilestorage::browser::index_offline_html
  tilestorage/browser/index-offline.html)
file_to_cpp(vadstena-libs_BROWSER_SOURCES
  vadstena::tilestorage::browser::skydome_jpg
  tilestorage/browser/skydome.jpg)

add_library(vadstena-tileset-browser STATIC
  ${vadstena-libs_BROWSER_SOURCES}
  tilestorage/support.hpp tilestorage/support.cpp
  )
buildsys_library(vadstena-tileset-browser)

set(vadstena-tileset-core_SOURCES
  tilestorage/basetypes.hpp
  tilestorage/properties.hpp
  tilestorage/tileop.hpp tilestorage/tileop.cpp
  tilestorage/io.hpp
  tilestorage/json.hpp tilestorage/json.cpp
  tilestorage/metatile.hpp tilestorage/metatile.cpp

  tilestorage/driver.hpp tilestorage/driver/driver.cpp
  tilestorage/driver/fsbased.hpp tilestorage/driver/fsbased.cpp
  tilestorage/driver/flat.hpp
  tilestorage/driver/hash-crc.hpp tilestorage/driver/hash-crc.cpp
  tilestorage/driver/ro-base.hpp
  tilestorage/driver/tar.hpp tilestorage/driver/tar.cpp

  tilestorage/driver/tilardriver.hpp
  tilestorage/driver/tilardriver/tilardriver.cpp
  tilestorage/driver/tilardriver/options.hpp
  tilestorage/driver/tilardriver/cache.hpp
  tilestorage/driver/tilardriver/cache.cpp

  tilestorage/config.hpp tilestorage/config.cpp
  )

define_module(LIBRARY vadstena-tileset-core DEPENDS
  vadstena-libs-storage math utility dbglog jsoncpp Boost_IOSTREAMS)
add_library(vadstena-tileset-core STATIC ${vadstena-tileset-core_SOURCES})
buildsys_library(vadstena-tileset-core)
target_link_libraries(vadstena-tileset-core
  vadstena-tileset-browser
  ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(vadstena-tileset-core ${MODULE_DEFINITIONS})

# Create vadstena-vts-browser library with browser files
file_to_cpp(vadstena-libs_vts_BROWSER_SOURCES
  vadstena::vts::browser::index_html
  vts/browser/index.html)
file_to_cpp(vadstena-libs_vts_BROWSER_SOURCES
  vadstena::vts::browser::index_offline_html
  vts/browser/index-offline.html)
file_to_cpp(vadstena-libs_vts_BROWSER_SOURCES
  vadstena::vts::browser::skydome_jpg
  vts/browser/skydome.jpg)

add_library(vadstena-vts-browser STATIC
  ${vadstena-libs_vts_BROWSER_SOURCES}
  vts/support.hpp vts/support.cpp
  )
buildsys_library(vadstena-vts-browser)

set(vadstena-vts-core_SOURCES
  vts/basetypes.hpp
  vts/properties.hpp
  vts/tileop.hpp vts/tileop.cpp
  vts/io.hpp
  vts/json.hpp vts/json.cpp
  vts/metatile.hpp vts/metatile.cpp

  vts/driver.hpp vts/driver/driver.cpp

  vts/driver/tilardriver.hpp
  vts/driver/tilardriver/tilardriver.cpp
  vts/driver/tilardriver/options.hpp
  vts/driver/tilardriver/cache.hpp
  vts/driver/tilardriver/cache.cpp

  vts/config.hpp vts/config.cpp

  vts/referenceframe.hpp vts/referenceframe.cpp
  )

define_module(LIBRARY vadstena-vts-core DEPENDS
  vadstena-libs-storage math utility dbglog jsoncpp half Boost_IOSTREAMS)
add_library(vadstena-vts-core STATIC ${vadstena-vts-core_SOURCES})
buildsys_library(vadstena-vts-core)
target_link_libraries(vadstena-vts-core
  vadstena-vts-browser
  ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(vadstena-vts-core ${MODULE_DEFINITIONS})

if (NOT DEFINED BUILDSYS_NOBUILD_TARGET_vadstena-libs)
  if (BUILDSYS_CUSTOMER_BUILD)
    message(STATUS "Building vadstena-libs customized for customer "
      "'${BUILDSYS_CUSTOMER}'.")
    set(CUSTOMER_SOURCES customer/${BUILDSYS_CUSTOMER}.cpp)

    # fingerprint:
    include(${CMAKE_CURRENT_LIST_DIR}/licence/fingerprint.cmake)
    fingerprint_to_cpp(CUSTOMER_SOURCES customer/${BUILDSYS_CUSTOMER}.fp
      customer/base.fp)
  else()
    message(STATUS "Building vadstena-libs without any customization.")
    set(CUSTOMER_SOURCES customer/none.cpp)
  endif()

  file_to_cpp(PUBLIC_KEY_SOURCES vadstena::matrixSource detail/matrix-source)
  file_to_cpp(PUBLIC_KEY_SOURCES vadstena::fingerPrintManipulator
    detail/fingeprint-manipulator)

  define_module(LIBRARY vadstena-libs DEPENDS
    vadstena-tileset-core
    vadstena-vts-core
    service cvision optics math geo geometry utility dbglog jsoncpp
    Boost_FILESYSTEM Boost_REGEX
    OpenCV LIBSHP CRYPTO++ MYSQL++)

  set(vadstena-libs_SOURCES
    definitions.hpp imagetypes.hpp
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
    colortrans.hpp
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
    keepalive.hpp keepalive.cpp

    cameralist.hpp cameralist.cpp
    facetexcoords.hpp facetexcoords.cpp
    gil2cvmat.hpp
    gcp.hpp gcp.cpp

    rectalloc.hpp rectalloc.cpp
    viewrasterize.hpp viewrasterize.cpp
    viewquality.hpp viewquality.cpp
    orthopreview.hpp orthopreview.cpp

    meshrasterize.hpp meshrasterize.cpp
    coverage-map.hpp coverage-map.cpp

    shapefile.hpp shapefile.cpp

    viewconfig.hpp viewconfig.cpp
    flowconfig.hpp flowconfig.cpp
    features.hpp features.cpp
    filelist.hpp filelist.cpp

    measurements.hpp measurements.cpp

    facespertile.hpp facespertile.cpp

    lodmapping.hpp

    tilestorage.hpp
    tilestorage/types.hpp
    tilestorage/storage.hpp tilestorage/storage.cpp
    tilestorage/tileset.hpp tilestorage/tileset.cpp
    tilestorage/po.hpp
    tilestorage/tileindex.hpp tilestorage/tileindex.cpp
    tilestorage/metatileop.hpp tilestorage/metatileop.cpp

    tilestorage/merge.hpp tilestorage/merge.cpp
    tilestorage/tileset/merge.cpp
    tilestorage/tileset/dump.hpp
    tilestorage/tileset/heightmap.cpp

    vts.hpp
    vts/types.hpp
    vts/tileset.hpp vts/tileset.cpp
    vts/po.hpp
    vts/tileindex.hpp vts/tileindex.cpp
    vts/metatileop.hpp vts/metatileop.cpp

    vts/tileset/merge.cpp
    vts/tileset/dump.hpp

    licence/fingerprint.hpp licence/fingerprint.cpp

    ${PUBLIC_KEY_SOURCES}

    customer.hpp ${CUSTOMER_SOURCES}
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
