# ------------------------------------------------------------------------
# VTS0 to VTS conversion tool
# ------------------------------------------------------------------------
define_module(BINARY vts02vts
  DEPENDS JPEG vts-libs)

set(vts02vts_SOURCES
  vts02vts.cpp
  heightmap.cpp
  )

vadstena_tool(storage vts02vts ${vts02vts_SOURCES})

# ------------------------------------------------------------------------
# VTS to VTS conversion tool
# ------------------------------------------------------------------------
define_module(BINARY vts2vts
  DEPENDS JPEG vts-libs)

set(vts2vts_SOURCES
  vts2vts.cpp
  heightmap.cpp
  )

vadstena_tool(storage vts2vts ${vts2vts_SOURCES})

# force build of vts tool
add_custom_target(.vts ALL DEPENDS vts)
