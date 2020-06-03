# VTS libraries
# builds: core and main library

# nucleus library: minimal subset only required for browser (renderer)
#
# core library: allows tileset access, for example in delivery machinery;
# depends on as little as possible dependencies (boost, gdal, proj)
#
# main library: allows better tileset data manipulation; uses core library;
# major dependency is OpenCV

# bump version here
set(vts-libs_VERSION 2.16)

# settable path to VTS registry
set(VTS_LIBS_REGISTRY_PATH "${CMAKE_INSTALL_PREFIX}/etc/registry" CACHE PATH
  "Installation path to VTS registry")

# generate config file
message(STATUS "Generatig VTS configuration.")
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/registry/config.cpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/registry/config.cpp
  ESCAPE_QUOTES)

if(VTSLIBS_BROWSER_ONLY)
  buildsys_nobuild_target(vts-libs-core:vts0)
  buildsys_nobuild_target(vts-libs:vts0)
  buildsys_nobuild_target(vts-libs)
endif()

# common stuff
set(vts-common_SOURCES
  storage/lod.hpp storage/range.hpp
  storage/error.hpp
  storage/openfiles.hpp storage/openfiles.cpp
  storage/filetypes.hpp
  storage/streams.hpp storage/streams.cpp
  storage/fstreams.hpp storage/fstreams.cpp
  storage/sstreams.hpp storage/sstreams.cpp
  )

if(NOT VTSLIBS_BROWSER_ONLY)
  list(APPEND vts-common_SOURCES
    storage/support.hpp storage/support.cpp
    )
endif()

# tilar not available on windows, sorry
if(NOT (WIN32 OR VTSLIBS_BROWSER_ONLY))
  list(APPEND vts-common_SOURCES
    storage/tilar.hpp storage/tilar.cpp
    storage/tilar-io.hpp
    )
endif()

# registry
set(vts-registry_SOURCES
  registry.hpp registry/registry.cpp
  registry/referenceframe.hpp registry/referenceframe.cpp
  registry/freelayer.hpp registry/freelayer.cpp
  registry/service.hpp registry/service.cpp
  registry/urlexpander.hpp registry/urlexpander.cpp
  registry/detail/json.hpp registry/detail/json.cpp
  ${CMAKE_CURRENT_BINARY_DIR}/registry/config.cpp
  registry/extensions.hpp registry/extensions.cpp
  )

if(NOT VTSLIBS_BROWSER_ONLY)
  list(APPEND vts-registry_SOURCES
    registry/json.hpp registry/dict.hpp
    registry/po.hpp registry/po.cpp
    registry/io.hpp registry/io.cpp
    registry/extensions/tms.hpp registry/extensions/tms.cpp
    registry/extensions/wmts.hpp registry/extensions/wmts.cpp
    )
endif()

if (NOT DEFINED BUILDSYS_NOBUILD_TARGET_vts-libs-core)
  if(VTS_REGISTRY_PYTHON AND NOT VTSLIBS_BROWSER_ONLY)
    message(STATUS "vts-libs-core: compiling in python support")
    set(vts-registry_python_DEPENDS
      pysupport pydbglog Boost_PYTHON)
    set(vts-registry_python_SOURCES
      registry/py.hpp registry/py.cpp)
  else()
    message(STATUS "vts-libs-core: compiling without python support")
    set(vts-registry_python_DEPENDS)
    set(vts-registry_python_SOURCES)
  endif()
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
if(NOT VTSLIBS_BROWSER_ONLY)
  set(vts_BROWSER_SOURCES
    vts/support.hpp vts/support.cpp
    )
  file_to_cpp(vts_BROWSER_SOURCES
    vtslibs::vts::browser::index_html
    vts/browser/index.html)
  file_to_cpp(vts_BROWSER_SOURCES
    vtslibs::vts::browser::skydome_jpg
    vts/browser/skydome.jpg)
  file_to_cpp(vts_BROWSER_SOURCES
    vtslibs::vts::browser::debugger_html
    vts/browser/debugger.html)
  file_to_cpp(vts_BROWSER_SOURCES
    vtslibs::vts::browser::debugger_js
    vts/browser/debugger.js)
endif()

# vts core files
set(vts-core_SOURCES
  vts/basetypes.hpp vts/basetypes.cpp
  vts/basetypes3.hpp
  vts/glue.hpp
  vts/nodeinfo.hpp vts/nodeinfo.cpp
  vts/mesh.hpp vts/mesh.cpp
  vts/meshio.hpp vts/meshio.cpp
  vts/metatile.hpp vts/metatile.cpp
  vts/csconvertor.hpp vts/csconvertor.cpp
  vts/math.hpp
  vts/visit.hpp
  vts/maskfwd.hpp
  vts/qtree.hpp vts/qtree.cpp
  vts/tileop.hpp vts/tileop.cpp
  vts/tileop3.hpp
  vts/metaflags.hpp vts/metaflags.cpp
  vts/mapconfig.hpp vts/mapconfig.cpp
  vts/types2d.hpp
  vts/urltemplate.hpp vts/urltemplate.cpp
  vts/urltemplate-po.hpp
  vts/geomextents.hpp vts/geomextents.cpp
  vts/virtualsurface.cpp
  vts/tsmap.hpp vts/tsmap.cpp
  vts/atmospheredensitytexture.hpp vts/atmospheredensitytexture.cpp
  )

if(NOT VTSLIBS_BROWSER_ONLY)
  list(APPEND vts-core_SOURCES
    vts.hpp vts/vts.cpp

    vts/config.hpp vts/config.cpp

    vts/meshcoverage.cpp
    vts/options.hpp vts/options.cpp
    vts/encoder.hpp vts/encoder.cpp
    vts/tileindex.hpp vts/tileindex.cpp vts/tileindex-io.hpp
    vts/tileflags.hpp vts/tileflags.cpp
    vts/encodeflags.hpp vts/encodeflags.cpp
    vts/gil/colors.hpp vts/gil/colors.cpp
    vts/debug.hpp vts/debug.cpp
    vts/2d.hpp vts/2d.cpp
    vts/multifile.hpp vts/multifile.cpp
    vts/navtile.hpp vts/navtile.cpp
    vts/atlas.hpp vts/atlas.cpp
    vts/subtrees.hpp vts/subtrees.cpp

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
    vts/tileset/driver/asyncmeta.cpp
    vts/tileset/driver/remote.hpp vts/tileset/driver/remote.cpp
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

    vts/service.hpp vts/service.cpp
    )
endif()



set(vts-core_EXTRA_DEPENDS)

if(MODULE_http_FOUND)
  message(STATUS "vts-core: compiling in HTTP support")
  list(APPEND vts-core_SOURCES
    vts/tileset/driver/httpfetcher.hpp vts/tileset/driver/httpfetcher.cpp)
  list(APPEND vts-core_EXTRA_DEPENDS http>=1.8)
else()
  list(APPEND vts-core_SOURCES
    vts/tileset/driver/httpfetcher.hpp vts/tileset/driver/httpfetcher.dummy.cpp)
  message(STATUS "vts-core: compiling without HTTP support")
endif()

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
  # glue/merge stuff
  vts/tileset/glue.cpp
  vts/tileset/merge.hpp vts/tileset/merge.cpp
  vts/tileset/merge/support.hpp vts/tileset/merge/support.cpp
  vts/tileset/merge/coverage.hpp vts/tileset/merge/coverage.cpp
  vts/tileset/merge/output.cpp

  # mesh operations
  vts/meshopinput.hpp vts/meshopinput.cpp
  vts/meshop.hpp
  vts/meshop/refineandclip.cpp
  vts/meshop/merge.cpp
  vts/meshop/skirt.cpp
  vts/meshop/optimize.cpp

  # storage
  vts/storage/change.cpp
  vts/storage/locking.hpp vts/storage/locking.cpp

  # dump support
  vts/tileindex-dump.cpp
  vts/qtree-dump.cpp

  # heightmap-related stuff
  vts/heightmap.hpp vts/heightmap.cpp
  vts/ntgenerator.hpp vts/ntgenerator.cpp

  # various stuff
  vts/options-po.hpp
  )

if(NOT VTSLIBS_BROWSER_ONLY)
  list(APPEND vts-core_EXTRA_DEPENDS
    CURL PNG
    )
endif()

# vts core library
define_module(LIBRARY vts-libs-core=${vts-libs_VERSION}
  DEPENDS
  geo>=1.33 geometry>=1.7 imgproc>=1.27 math>=1.7
  utility>=1.30 dbglog>=1.4 jsoncpp>=2.1 half>=1.1

  Boost_IOSTREAMS Boost_FILESYSTEM

  ${vts-registry_python_DEPENDS}
  ${vts-core_EXTRA_DEPENDS}
  )

add_library(vts-libs-core STATIC
  ${vts-common_SOURCES}
  ${vts-registry_SOURCES}
  ${vts-nucleus_SOURCES}

  ${vts-registry_python_SOURCES}

  ${vts_BROWSER_SOURCES}
  ${vts-core_SOURCES}

  ${vts0_BROWSER_SOURCES}
  ${vts0-core_SOURCES}
  )
buildsys_library(vts-libs-core)
target_link_libraries(vts-libs-core ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(vts-libs-core ${MODULE_DEFINITIONS})

if(VTSLIBS_BROWSER_ONLY)
  target_compile_definitions(vts-libs-core PRIVATE VTSLIBS_BROWSER_ONLY)
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
    )
  buildsys_library(vts-libs)
  target_link_libraries(vts-libs ${MODULE_LIBRARIES})
  buildsys_target_compile_definitions(vts-libs ${MODULE_DEFINITIONS})
endif()


if(NOT DEFINED BUILDSYS_NOBUILD_TARGET_vts-libs-tools-support AND TARGET vts-libs)
  message(STATUS "Compiling vts-libs-tools library.")
  set(vts-libs-tools-support_EXTRA_DEPENDS )
  set(vts-libs-tools-support_EXTRA_SOURCES )

  if (ASSIMP_FOUND AND MODULE_roarchive_FOUND)
    list(APPEND vts-libs-tools-support_EXTRA_DEPENDS roarchive ASSIMP)
     list(APPEND vts-libs-tools-support_EXTRA_SOURCES
      tools-support/assimp.hpp tools-support/assimp.cpp
      )
  endif()

  # support library for VTS tools
  define_module(LIBRARY vts-libs-tools-support=${vts-libs_VERSION}
    DEPENDS vts-libs ${vts-libs-tools-support_EXTRA_DEPENDS}
    )

  set(vts-libs-tools-support_SOURCES
    # TODO: copy stuff from vts-tools repository
    tools-support/progress.hpp tools-support/progress.cpp

    tools-support/analyze.cpp tools-support/analyze.hpp
    tools-support/repackatlas.cpp tools-support/repackatlas.hpp
    tools-support/tmptileset.cpp tools-support/tmptileset.hpp
    tools-support/tmptsencoder.cpp tools-support/tmptsencoder.hpp
    tools-support/optimizemesh.cpp tools-support/optimizemesh.hpp
    )

  add_library(vts-libs-tools-support
    STATIC ${vts-libs-tools-support_SOURCES}
    ${vts-libs-tools-support_EXTRA_SOURCES})
  buildsys_library(vts-libs-tools-support)
  target_link_libraries(vts-libs-tools-support ${MODULE_LIBRARIES})
  buildsys_target_compile_definitions(vts-libs-tools-support ${MODULE_DEFINITIONS})
endif()

# add tools subdirectory
if(MODULE_service_FOUND AND TARGET vts-libs)
  if (vts-libs_vts-install_component)
    add_subdirectory(tools)
  else()
    add_subdirectory(tools EXCLUDE_FROM_ALL)
  endif()
endif()
