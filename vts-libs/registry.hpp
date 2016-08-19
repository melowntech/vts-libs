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

/** Base registry: credits and bound layers only
 */
struct RegistryBase {
    BoundLayer::dict boundLayers;
    Credit::dict credits;

    RegistryBase() {};
};

/** Full registry
 */
struct Registry : RegistryBase {
    Srs::dict srs;
    ReferenceFrame::dict referenceFrames;

    Registry() = default;
    Registry(const Registry&) = default;
    Registry(const RegistryBase &base) : RegistryBase(base) {}
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

void load(RegistryBase &rb, const boost::filesystem::path &path);

void load(RegistryBase &rb, std::istream &in
          , const boost::filesystem::path &path
          = "UNKNOWN");

void save(const boost::filesystem::path &path, const RegistryBase &rb);

void save(std::ostream &out, const RegistryBase &rb);

} } // namespace vadstena::registry

#endif // vadstena_libs_registry_hpp_included_
