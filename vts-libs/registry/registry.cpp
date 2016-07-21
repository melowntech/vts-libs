#include "utility/buildsys.hpp"

#include "../registry.hpp"

namespace vadstena { namespace registry {

namespace detail {
    boost::filesystem::path root;

    DataFile::dict dataFilesByFilename;
    DataFile::dict dataFilesByPath;
} // namespace detail

Registry system;

void init(const boost::filesystem::path &confRoot)
{
    detail::root = confRoot;

    system.srs = loadSrs(confRoot / "srs.json");
    system.referenceFrames
        = loadReferenceFrames(confRoot / "referenceframes.json");
    system.boundLayers = loadBoundLayers(confRoot / "boundlayers.json");
    system.credits = loadCredits(confRoot / "credits.json");

    // grab geoid grid files from srs
    for (const auto &srs : system.srs) {
        if (!srs.second.geoidGrid) { continue; }
        const auto &path(srs.second.geoidGrid->definition);
        if (detail::dataFilesByPath.has(path)) { continue; }

        DataFile df(detail::root / path);
        detail::dataFilesByPath.set(path, df);
        detail::dataFilesByFilename.set
            (boost::filesystem::path(path).filename().string(), df);
    }
}

boost::filesystem::path confRoot()
{
    return detail::root;
}

const DataFile& dataFile(const std::string &path, DataFile::Key key)
{
    switch (key) {
    case DataFile::Key::filename:
        return detail::dataFilesByFilename.get(path);

    case DataFile::Key::path:
        return detail::dataFilesByPath.get(path);
    }
    throw; // never reached
}

const DataFile* dataFile(const std::string &path, DataFile::Key key
                         , std::nothrow_t)
{
    switch (key) {
    case DataFile::Key::filename:
        return detail::dataFilesByFilename.get(path, std::nothrow);

    case DataFile::Key::path:
        return detail::dataFilesByPath.get(path, std::nothrow);
    }
    throw; // never reached
}

} } // namespace vadstena::registry
