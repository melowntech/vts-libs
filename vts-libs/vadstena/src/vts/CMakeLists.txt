# ------------------------------------------------------------------------
# VTS query and manipulation tool
# ------------------------------------------------------------------------
define_module(BINARY vts
  DEPENDS vadstena-libs)

set(vts_SOURCES
  vts.cpp
  )

vadstena_tool(storage vts ${vts_SOURCES})

# ------------------------------------------------------------------------
# VTS0 to VTS conversion tool
# ------------------------------------------------------------------------
define_module(BINARY vts02vts
  DEPENDS JPEG vadstena-libs)

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

# ------------------------------------------------------------------------
# Helper program to fix flipped mask from vadstena-heightfield
# (to be decomissioned)
# ------------------------------------------------------------------------
define_module(BINARY flipmeshmask
  DEPENDS JPEG  vts-libs)

set(flipmeshmask_SOURCES
  flipmeshmask.cpp
  )

#vadstena_tool(storage flipmeshmask ${flipmeshmask_SOURCES})

# ------------------------------------------------------------------------
# Helper program to fix wrong grand (to be decomissioned)
# ------------------------------------------------------------------------
define_module(BINARY fixphysicalmesh
  DEPENDS JPEG vts-libs)

set(fixphysicalmesh_SOURCES
  fixphysicalmesh.cpp
  )

#vadstena_tool(storage fixphysicalmesh ${fixphysicalmesh_SOURCES})
