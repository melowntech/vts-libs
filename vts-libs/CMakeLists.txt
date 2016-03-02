# VTS libraries
# builds: core and main library

# core library: allows tileset access, for example in delivery machinery;
# depends on as little as possible dependencies (boost, gdal, proj)
#
# main library: allows better tileset data manipulation; uses core library;
# major dependency is OpenCV

# settable path to VTS registry
set(VTS_LIBS_REGISTRY_PATH "${CMAKE_INSTALL_PREFIX}/etc/registry" CACHE PATH
  "Installation path to VTS registry")

# generate config file
message(STATUS "Generatig VTS configuration.")
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/registry/config.cpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/registry/config.cpp
  ESCAPE_QUOTES)

# common stuff
set(vts-common_SOURCES
  storage/lod.hpp storage/range.hpp

  storage/error.hpp
  storage/openfiles.hpp storage/openfiles.cpp
  storage/filetypes.hpp
  storage/streams.hpp storage/streams.cpp
  storage/fstreams.hpp storage/fstreams.cpp

  storage/tilar.hpp storage/tilar.cpp
  storage/tilar-io.hpp

  storage/support.hpp
  )

# registry
set(vts-registry_SOURCES
  registry.hpp registry/registry.cpp
  registry/referenceframe.hpp registry/referenceframe.cpp
  ${CMAKE_CURRENT_BINARY_DIR}/registry/config.cpp
)


# old tilestorage browser files
set(tilestorage_BROWSER_SOURCES
  tilestorage/support.hpp tilestorage/support.cpp
  )
file_to_cpp(tilestorage_BROWSER_SOURCES
  vadstena::tilestorage::browser::index_html
  tilestorage/browser/index.html)
file_to_cpp(tilestorage_BROWSER_SOURCES
  vadstena::tilestorage::browser::index_offline_html
  tilestorage/browser/index-offline.html)
file_to_cpp(tilestorage_BROWSER_SOURCES
  vadstena::tilestorage::browser::skydome_jpg
  tilestorage/browser/skydome.jpg)


# old tilestorage core files
set(tilestorage-core_SOURCES
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

set(tilestorage_SOURCES
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
  )

# old vts0 browser files
set(vts0_BROWSER_SOURCES
  vts0/support.hpp vts0/support.cpp
  )
file_to_cpp(vts0_BROWSER_SOURCES
  vadstena::vts0::browser::index_html
  vts0/browser/index.html)
file_to_cpp(vts0_BROWSER_SOURCES
  vadstena::vts0::browser::index_offline_html
  vts0/browser/index-offline.html)
file_to_cpp(vts0_BROWSER_SOURCES
  vadstena::vts0::browser::skydome_jpg
  vts0/browser/skydome.jpg)

# old vts0 files
set(vts0-core_SOURCES
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


set(vts0_SOURCES
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
  )

# vts browser files
set(vts_BROWSER_SOURCES
  vts/support.hpp vts/support.cpp
  )
file_to_cpp(vts_BROWSER_SOURCES
  vadstena::vts::browser::index_html
  vts/browser/index.html)
file_to_cpp(vts_BROWSER_SOURCES
  vadstena::vts::browser::index_offline_html
  vts/browser/index-offline.html)
file_to_cpp(vts_BROWSER_SOURCES
  vadstena::vts::browser::skydome_jpg
  vts/browser/skydome.jpg)

# vts core files
set(vts-core_SOURCES
  vts.hpp vts/vts.cpp
  vts/basetypes.hpp vts/basetypes.cpp
  vts/multifile.hpp vts/multifile.cpp
  vts/mesh.hpp vts/mesh.cpp
  vts/metatile.hpp vts/metatile.cpp
  vts/atlas.hpp vts/atlas.cpp
  vts/navtile.hpp vts/navtile.cpp
  vts/encoder.hpp vts/encoder.cpp
  vts/csconvertor.hpp vts/csconvertor.cpp

  vts/maskfwd.hpp
  vts/qtree.hpp vts/qtree.cpp
  vts/tileindex.hpp vts/tileindex.cpp vts/tileindex-io.hpp
  vts/tileop.hpp vts/tileop.cpp

  vts/tileset.hpp
  vts/tileset/detail.hpp
  vts/tileset/properties.hpp
  vts/tileset/tileset.cpp
  vts/tileset/config.hpp vts/tileset/config.cpp

  vts/tileset/driver.hpp
  vts/tileset/driver/driver.cpp
  vts/tileset/driver/options.hpp
  vts/tileset/driver/cache.hpp vts/tileset/driver/cache.cpp
  vts/tileset/driver/plain.hpp vts/tileset/driver/plain.cpp

  vts/storage.hpp
  vts/storage/detail.hpp
  vts/storage/storage.cpp
  vts/storage/config.hpp vts/storage/config.cpp

  vts/storageview.hpp
  vts/storageview/detail.hpp
  vts/storageview/storageview.cpp
  vts/storageview/config.hpp vts/storageview/config.cpp

  vts/mapconfig.hpp vts/mapconfig.cpp
  )

set(vts_SOURCES
  vts/opencv/atlas.hpp vts/opencv/atlas.cpp
  vts/opencv/navtile.hpp vts/opencv/navtile.cpp
  vts/tileset/glue.cpp
  vts/tileset/merge.hpp vts/tileset/merge.cpp
  vts/meshop.hpp vts/meshopinput.hpp
  vts/meshop/refineandclip.cpp
  vts/meshop/merge.cpp
  vts/storage/change.cpp

  # dump support
  vts/tileindex-dump.cpp
  vts/qtree-dump.cpp
  )

# vts core library
define_module(LIBRARY vts-libs-core DEPENDS
  geo imgproc math utility dbglog jsoncpp half
  Boost_IOSTREAMS
  Boost_FILESYSTEM
  )

add_library(vts-libs-core STATIC
  ${vts-common_SOURCES}
  ${vts-registry_SOURCES}

  ${vts_BROWSER_SOURCES}
  ${vts-core_SOURCES}

  ${vts0_BROWSER_SOURCES}
  ${vts0-core_SOURCES}

  ${tilestorage_BROWSER_SOURCES}
  ${tilestorage-core_SOURCES}
  )
buildsys_library(vts-libs-core)
target_link_libraries(vts-libs-core
  ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(vts-libs-core ${MODULE_DEFINITIONS})

# main vts library is optional:
if (NOT DEFINED BUILDSYS_NOBUILD_TARGET_vts-libs)
  message(STATUS "Compiling main vts-libs library.")
  define_module(LIBRARY vts-libs DEPENDS
    vts-libs-core
    service cvision optics
    OpenCV)


  add_library(vts-libs STATIC
    ${vts_SOURCES}
    ${vts0_SOURCES}
    ${tilestorage_SOURCES}
    )
  buildsys_library(vts-libs)
  target_link_libraries(vts-libs ${MODULE_LIBRARIES})
  buildsys_target_compile_definitions(vts-libs ${MODULE_DEFINITIONS})
endif()

# add tools subdirectory
if(MODULE_service_FOUND)
  add_subdirectory(tools EXCLUDE_FROM_ALL)
endif()
