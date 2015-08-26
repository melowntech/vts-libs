define_module(BINARY ts2vts
  DEPENDS vadstena-libs)

# ------------------------------------------------------------------------
set(ts2vts_SOURCES
  ts2vts.cpp
  )

vadstena_tool(storage ts2vts ${ts2vts_SOURCES})

# ------------------------------------------------------------------------
set(vts-dumpindex_SOURCES
  dumpindex.cpp
  )

vadstena_tool(storage vts-dumpindex ${vts-dumpindex_SOURCES})

vadstena_tool(storage vts-dump-metatile dump-metatile.cpp)

vadstena_tool(storage vts-loadreferenceframes loadreferenceframes.cpp)
