define_module(BINARY vts
  DEPENDS vadstena-libs)

# ------------------------------------------------------------------------
set(vts_SOURCES
  vts.cpp
  )

vadstena_tool(storage vts ${vts_SOURCES})
