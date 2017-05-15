# VTS libraries
# builds: core and main library

# nukleus library: minimal subset only required for browser (renderer)
#
# core library: allows tileset access, for example in delivery machinery;
# depends on as little as possible dependencies (boost, gdal, proj)
#
# main library: allows better tileset data manipulation; uses core library;
# major dependency is OpenCV

# bump version here
set(vts-libs_VERSION 2.4)

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
  storage/sstreams.hpp storage/sstreams.cpp

  storage/tilar.hpp storage/tilar.cpp
  storage/tilar-io.hpp

  storage/support.hpp storage/support.cpp
  )

# registry
set(vts-registry_SOURCES
  registry.hpp registry/registry.cpp
  registry/referenceframe.hpp registry/referenceframe.cpp
  registry/freelayer.hpp registry/freelayer.cpp
  registry/json.hpp registry/dict.hpp
  registry/po.hpp registry/po.cpp
  registry/io.hpp
  ${CMAKE_CURRENT_BINARY_DIR}/registry/config.cpp
  )

set(vts-registry_python_DEPENDS)
if(VTS_REGISTRY_PYTHON)
  message(STATUS "vts-libs-core: compiling in python support")
  list(APPEND vts-registry_python_DEPENDS
    pysupport pydbglog Boost_PYTHON)
  set(vts-registry_python_SOURCES
    registry/py.hpp registry/py.cpp
    )
else()
  message(STATUS "vts-libs-core: compiling without python support")
  set(vts-registry_python_SOURCES)
endif()

if(NOT DEFINED BUILDSYS_NOBUILD_TARGET_vts-libs-core:tilestorage)
  # old tilestorage browser files
  set(tilestorage_BROWSER_SOURCES
    tilestorage/support.hpp tilestorage/support.cpp
    )
  file_to_cpp(tilestorage_BROWSER_SOURCES
    vtslibs::tilestorage::browser::index_html
    tilestorage/browser/index.html)
  file_to_cpp(tilestorage_BROWSER_SOURCES
    vtslibs::tilestorage::browser::index_offline_html
    tilestorage/browser/index-offline.html)
  file_to_cpp(tilestorage_BROWSER_SOURCES
    vtslibs::tilestorage::browser::skydome_jpg
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
else()
  message(STATUS "vts-libs-core: compiling without tilestorage stuff")
  set(tilestorage_BROWSER_SOURCES)
  set(tilestorage-core_SOURCES)

  # disable main tilestorage stuff as well
  buildsys_nobuild_target(vts-libs:tilestorage)
endif()

if(NOT DEFINED BUILDSYS_NOBUILD_TARGET_vts-libs:tilestorage)
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
else()
  message(STATUS "vts-libs: compiling without tilestorage stuff")
  set(tilestorage_SOURCES)
endif()

if (NOT DEFINED BUILDSYS_NOBUILD_TARGET_vts-libs-core:vts0)
  # old vts0 browser files
  set(vts0_BROWSER_SOURCES
    vts0/support.hpp vts0/support.cpp
    )
  file_to_cpp(vts0_BROWSER_SOURCES
    vtslibs::vts0::browser::index_html
    vts0/browser/index.html)
  file_to_cpp(vts0_BROWSER_SOURCES
    vtslibs::vts0::browser::index_offline_html
    vts0/browser/index-offline.html)
  file_to_cpp(vts0_BROWSER_SOURCES
    vtslibs::vts0::browser::skydome_jpg
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
else()
  message(STATUS "vts-libs-core: compiling without vts0 stuff")
  set(vts0_BROWSER_SOURCES)
  set(vts0-core_SOURCES)

  # disable main vts stuff as well
  buildsys_nobuild_target(vts-libs:vts0)
endif()

if(NOT DEFINED BUILDSYS_NOBUILD_TARGET_vts-libs:vts0)
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
else()
  message(STATUS "vts-libs: compiling without vts0 stuff")
  set(vts0_SOURCES)
endif()

# vts browser files
set(vts_BROWSER_SOURCES
  vts/support.hpp vts/support.cpp
  )
file_to_cpp(vts_BROWSER_SOURCES
  vtslibs::vts::browser::index_html
  vts/browser/index.html)
file_to_cpp(vts_BROWSER_SOURCES
  vtslibs::vts::browser::index_offline_html
  vts/browser/index-offline.html)
file_to_cpp(vts_BROWSER_SOURCES
  vtslibs::vts::browser::skydome_jpg
  vts/browser/skydome.jpg)
file_to_cpp(vts_BROWSER_SOURCES
  vtslibs::vts::browser::debugger_html
  vts/browser/debugger.html)
file_to_cpp(vts_BROWSER_SOURCES
  vtslibs::vts::browser::debugger_js
  vts/browser/debugger.js)

# vts nukleus files
set(vts-nukleus_SOURCES
  vts.hpp vts/vts.cpp
  vts/basetypes.hpp vts/basetypes.cpp
  vts/options.hpp vts/options.cpp
  vts/glue.hpp
  vts/nodeinfo.hpp vts/nodeinfo.cpp
  vts/multifile.hpp vts/multifile.cpp
  vts/mesh.hpp vts/mesh.cpp
  vts/meshio.hpp vts/meshio.cpp
  vts/metatile.hpp vts/metatile.cpp
  vts/atlas.hpp vts/atlas.cpp
  vts/navtile.hpp vts/navtile.cpp
  vts/encoder.hpp vts/encoder.cpp
  vts/csconvertor.hpp vts/csconvertor.cpp
  vts/math.hpp
  vts/subtrees.hpp vts/subtrees.cpp
  vts/visit.hpp
  vts/maskfwd.hpp
  vts/qtree.hpp vts/qtree.cpp
  vts/tileindex.hpp vts/tileindex.cpp vts/tileindex-io.hpp
  vts/tileop.hpp vts/tileop.cpp
  vts/tileflags.hpp vts/tileflags.cpp
  vts/metaflags.hpp vts/metaflags.cpp
  vts/encodeflags.hpp vts/encodeflags.cpp
  vts/mapconfig.hpp vts/mapconfig.cpp
  vts/types2d.hpp
  vts/2d.hpp vts/2d.cpp
  vts/urltemplate.hpp vts/urltemplate.cpp
  vts/gil/colors.hpp vts/gil/colors.cpp
  vts/debug.hpp vts/debug.cpp
  vts/geomextents.hpp vts/geomextents.cpp
  vts/virtualsurface.cpp
  )

# vts core files
set(vts-core_SOURCES
  vts/meshcoverage.cpp

  vts/tileset.hpp
  vts/tileset/detail.hpp
  vts/tileset/properties.hpp
  vts/tileset/tileset.cpp
  vts/tileset/config.hpp vts/tileset/config.cpp
  vts/tileset/tilesetindex.hpp vts/tileset/tilesetindex.cpp
  vts/tileset/metacache.hpp
  vts/tileset/metacache/ro.cpp vts/tileset/metacache/rw.cpp

  vts/tileset/driver.hpp
  vts/tileset/driver/driver.cpp
  vts/tileset/driver/options.hpp
  vts/tileset/driver/cache.hpp vts/tileset/driver/cache.cpp
  vts/tileset/driver/plain.hpp vts/tileset/driver/plain.cpp
  vts/tileset/driver/aggregated.hpp vts/tileset/driver/aggregated.cpp
  vts/tileset/driver/remote.hpp vts/tileset/driver/remote.cpp
  vts/tileset/driver/httpfetcher.hpp vts/tileset/driver/httpfetcher.cpp
  vts/tileset/driver/local.hpp vts/tileset/driver/local.cpp

  vts/tileset/delivery.hpp vts/tileset/driver/delivery.cpp

  vts/storage.hpp
  vts/storage/detail.hpp
  vts/storage/storage.cpp
  vts/storage/config.hpp vts/storage/config.cpp
  vts/storage/gluerules.hpp vts/storage/gluerules.cpp
  vts/storage/mergeconf.hpp vts/storage/mergeconf.cpp
  vts/storage/locking.hpp vts/storage/locking.cpp

  vts/storageview.hpp
  vts/storageview/detail.hpp
  vts/storageview/storageview.cpp
  vts/storageview/config.hpp vts/storageview/config.cpp
  )

set(vts-core_EXTRA_DEPENDS)

if(OpenCV_FOUND)
  # OpenCV -- link in core opencv stuff
  list(APPEND vts-core_SOURCES
    vts/opencv/atlas.hpp vts/opencv/atlas.cpp
    vts/opencv/navtile.hpp vts/opencv/navtile.cpp
    vts/opencv/colors.hpp vts/opencv/colors.cpp
    vts/opencv/inpaint.cpp
    vts/opencv/texture.hpp vts/opencv/texture.cpp
    )
  list(APPEND vts-core_EXTRA_DEPENDS OpenCV)
else()
  list(APPEND vts-core_SOURCES
    vts/inpaint.cpp
    )
endif()

set(vts_SOURCES
  vts/tileset/glue.cpp
  vts/tileset/merge.hpp vts/tileset/merge.cpp
  vts/meshop.hpp vts/meshopinput.hpp
  vts/meshop/refineandclip.cpp
  vts/meshop/merge.cpp
  vts/storage/change.cpp
  vts/storage/locking.hpp vts/storage/locking.cpp

  # dump support
  vts/tileindex-dump.cpp
  vts/qtree-dump.cpp

  # heightmap-related stuff
  vts/heightmap.hpp vts/heightmap.cpp
  vts/ntgenerator.hpp vts/ntgenerator.cpp
  )


# vts nukleus library
define_module(LIBRARY vts-libs-nukleus=${vts-libs_VERSION}
  DEPENDS
  http>=1.1 geo>=1.24 geometry>=1.7 imgproc>=1.19 math>=1.2
  utility>=1.16 dbglog>=1.4 jsoncpp>=2.1 half>=1.1

  Boost_IOSTREAMS
  Boost_FILESYSTEM
  CURL PNG
  )

add_library(vts-libs-nukleus STATIC
  ${vts-common_SOURCES}
  ${vts-registry_SOURCES}
  ${vts-nukleus_SOURCES}
  )
buildsys_library(vts-libs-nukleus)
target_link_libraries(vts-libs-nukleus ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(vts-libs-nukleus ${MODULE_DEFINITIONS})


# vts core library
if (NOT DEFINED BUILDSYS_NOBUILD_TARGET_vts-libs-core)
  define_module(LIBRARY vts-libs-core=${vts-libs_VERSION}
    DEPENDS
    vts-libs-nukleus>=${vts-libs_VERSION}
  
    ${vts-registry_python_DEPENDS}
    ${vts-core_EXTRA_DEPENDS}
    )
  
  add_library(vts-libs-core STATIC
    ${vts-registry_python_SOURCES}
  
    ${vts-nukleus_SOURCES}
  
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
endif()

# main vts library is optional:
if (NOT DEFINED BUILDSYS_NOBUILD_TARGET_vts-libs)
  message(STATUS "Compiling main vts-libs library.")
  define_module(LIBRARY vts-libs=${vts-libs_VERSION}
    DEPENDS
    vts-libs-core>=${vts-libs_VERSION}
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
  if (vts-libs_vts-install_component)
    add_subdirectory(tools)
  else()
    add_subdirectory(tools EXCLUDE_FROM_ALL)
  endif()
endif()
