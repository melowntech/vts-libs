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

  storage/referenceframe.hpp storage/referenceframe.cpp
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

# Create vadstena-vts0-browser library with browser files
file_to_cpp(vadstena-libs_vts0_BROWSER_SOURCES
  vadstena::vts0::browser::index_html
  vts0/browser/index.html)
file_to_cpp(vadstena-libs_vts0_BROWSER_SOURCES
  vadstena::vts0::browser::index_offline_html
  vts0/browser/index-offline.html)
file_to_cpp(vadstena-libs_vts0_BROWSER_SOURCES
  vadstena::vts0::browser::skydome_jpg
  vts0/browser/skydome.jpg)

add_library(vadstena-vts0-browser STATIC
  ${vadstena-libs_vts0_BROWSER_SOURCES}
  vts0/support.hpp vts0/support.cpp
  )
buildsys_library(vadstena-vts0-browser)

set(vadstena-vts0-core_SOURCES
  vts0/basetypes.hpp
  vts0/properties.hpp
  vts0/tileop.hpp vts0/tileop.cpp
  vts0/io.hpp
  vts0/json.hpp vts0/json.cpp
  vts0/metatile.hpp vts0/metatile.cpp

  vts0/driver.hpp vts0/driver/driver.cpp

  vts0/driver/tilardriver.hpp
  vts0/driver/tilardriver/tilardriver.cpp
  vts0/driver/tilardriver/options.hpp
  vts0/driver/tilardriver/cache.hpp
  vts0/driver/tilardriver/cache.cpp

  vts0/config.hpp vts0/config.cpp
  )

define_module(LIBRARY vadstena-vts0-core DEPENDS
  vadstena-libs-storage geo math utility dbglog jsoncpp half Boost_IOSTREAMS)
add_library(vadstena-vts0-core STATIC ${vadstena-vts0-core_SOURCES})
buildsys_library(vadstena-vts0-core)
target_link_libraries(vadstena-vts0-core
  vadstena-vts0-browser
  ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(vadstena-vts0-core ${MODULE_DEFINITIONS})

# VTS core library

set(vadstena-vts-core_SOURCES
  vts/mesh.hpp vts/mesh.cpp
  vts/metatile.hpp vts/metatile.cpp
  vts/atlas.hpp

  vts/tileset.hpp vts/tileset.cpp
  vts/tileset/detail.hpp
  vts/tileset/driver.hpp
  )

define_module(LIBRARY vadstena-vts-core DEPENDS
  vadstena-libs-storage geo math utility dbglog jsoncpp half Boost_IOSTREAMS)
add_library(vadstena-vts-core STATIC ${vadstena-vts-core_SOURCES})
buildsys_library(vadstena-vts-core)
target_link_libraries(vadstena-vts-core
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
    vadstena-vts0-core
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

    vts0.hpp
    vts0/types.hpp
    vts0/tileset.hpp vts0/tileset.cpp
    vts0/tileindex.hpp vts0/tileindex.cpp
    vts0/metatileop.hpp vts0/metatileop.cpp

    vts0/tileset/merge.cpp
    vts0/tileset/dump.hpp

    vts0/storage.hpp vts0/storage.cpp

    vts0/merge.hpp vts0/merge.cpp
    vts0/tileset/heightmap.cpp

    vts0/encoder.hpp vts0/encoder.cpp

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
