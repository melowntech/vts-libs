# ------------------------------------------------------------------------
define_module(BINARY vts
  DEPENDS vadstena-libs)

set(vts_SOURCES
  vts.cpp
  )

vadstena_tool(storage vts ${vts_SOURCES})

# ------------------------------------------------------------------------
define_module(BINARY vts02vts
  DEPENDS JPEG vadstena-libs)

set(vts02vts_SOURCES
  vts02vts.cpp
  heightmap.cpp
  )

vadstena_tool(storage vts02vts ${vts02vts_SOURCES})

# ------------------------------------------------------------------------
define_module(BINARY vts2vts
  DEPENDS JPEG vadstena-libs)

set(vts2vts_SOURCES
  vts2vts.cpp
  )

vadstena_tool(storage vts2vts ${vts2vts_SOURCES})
