/**
 * \file registry.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 */

#ifndef vadstena_libs_registry_hpp_included_
#define vadstena_libs_registry_hpp_included_

#include "./registry/referenceframe.hpp"
#include "./registry/freelayer.hpp"
#include "./registry/datafile.hpp"

namespace vadstena { namespace registry {

/** Registry: resource holder.
 */
class Registry {
public:
    Srs::dict srs;
    BoundLayer::dict boundLayers;
    ReferenceFrame::dict referenceFrames;
    Credit::dict credits;
};

/** System-wide registry
 */
extern Registry system;

void init(const boost::filesystem::path &confRoot);
boost::filesystem::path confRoot();

/** Returns default path to registry.
 *  NB: implemented in file generated from config.cpp.in template
 */
boost::filesystem::path defaultPath();

const DataFile& dataFile(const std::string &path, DataFile::Key key);
const DataFile* dataFile(const std::string &path, DataFile::Key key
                         , std::nothrow_t);

} } // namespace vadstena::registry

#endif // vadstena_libs_registry_hpp_included_
