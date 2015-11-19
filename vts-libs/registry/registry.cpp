#include "utility/buildsys.hpp"

#include "../registry.hpp"

namespace vadstena { namespace registry {

namespace detail {
    boost::filesystem::path root;

    Srs::dict srs;

    ReferenceFrame::dict referenceFrames;

    BoundLayer::dict boundLayers;
    BoundLayer::ndict nBoundLayers;

    Credit::dict credits;
    Credit::ndict nCredits;

    DataFile::dict dataFilesByFilename;
    DataFile::dict dataFilesByPath;
}

const Srs* Registry::srs(const std::string &id, std::nothrow_t)
{
    return detail::srs.get(id, std::nothrow);
}

const Srs& Registry::srs(const std::string &id)
{
    return detail::srs.get(id);
}

const Srs::dict Registry::srsList()
{
    return detail::srs;
}

const ReferenceFrame*
Registry::referenceFrame(const std::string &id, std::nothrow_t)
{
    return detail::referenceFrames.get(id, std::nothrow);
}

const ReferenceFrame& Registry::referenceFrame(const std::string &id)
{
    return detail::referenceFrames.get(id);
}

const ReferenceFrame::dict Registry::referenceFrames()
{
    return detail::referenceFrames;
}

const BoundLayer*
Registry::boundLayer(const std::string &id, std::nothrow_t)
{
    return detail::boundLayers.get(id, std::nothrow);
}

const BoundLayer& Registry::boundLayer(const std::string &id)
{
    return detail::boundLayers.get(id);
}

const BoundLayer*
Registry::boundLayer(BoundLayer::NumericId id, std::nothrow_t)
{
    return detail::nBoundLayers.get(id, std::nothrow);
}

const BoundLayer& Registry::boundLayer(BoundLayer::NumericId id)
{
    return detail::nBoundLayers.get(id);
}

const BoundLayer::dict Registry::boundLayers()
{
    return detail::boundLayers;
}

const BoundLayer::ndict Registry::boundLayers(int)
{
    return detail::nBoundLayers;
}

const Credit*
Registry::credit(const std::string &id, std::nothrow_t)
{
    return detail::credits.get(id, std::nothrow);
}

const Credit& Registry::credit(const std::string &id)
{
    return detail::credits.get(id);
}

const Credit*
Registry::credit(Credit::NumericId id, std::nothrow_t)
{
    return detail::nCredits.get(id, std::nothrow);
}

const Credit& Registry::credit(Credit::NumericId id)
{
    return detail::nCredits.get(id);
}

const Credit::dict Registry::credits()
{
    return detail::credits;
}

const Credit::ndict Registry::credits(int)
{
    return detail::nCredits;
}

const DataFile& Registry::dataFile(const std::string &path
                                   , DataFileKey key)
{
    switch (key) {
    case DataFileKey::filename:
        return detail::dataFilesByFilename.get(path);

    case DataFileKey::path:
        return detail::dataFilesByPath.get(path);
    }
    throw; // never reached
}

const DataFile* Registry::dataFile(const std::string &path
                                   , DataFileKey key
                                   , std::nothrow_t)
{
    switch (key) {
    case DataFileKey::filename:
        return detail::dataFilesByFilename.get(path, std::nothrow);

    case DataFileKey::path:
        return detail::dataFilesByPath.get(path, std::nothrow);
    }
    throw; // never reached
}

void Registry::init(const boost::filesystem::path &confRoot)
{
    detail::root = confRoot;

    detail::srs = loadSrs(confRoot / "srs.json");
    detail::referenceFrames
        = loadReferenceFrames(confRoot / "referenceframes.json");

    detail::boundLayers = loadBoundLayers(confRoot / "boundlayers.json");
    for (auto const &bl : detail::boundLayers) {
        detail::nBoundLayers.set(bl.second.numericId, bl.second);
    }

    detail::credits = loadCredits(confRoot / "credits.json");
    for (auto const &c : detail::credits) {
        detail::nCredits.set(c.second.numericId, c.second);
    }

    // grab geoid grid files from srs
    for (const auto &srs : detail::srs) {
        if (!srs.second.geoidGrid) { continue; }
        const auto &path(srs.second.geoidGrid->definition);
        if (detail::dataFilesByPath.has(path)) { continue; }

        DataFile df(detail::root / path);
        detail::dataFilesByPath.set(path, df);
        detail::dataFilesByFilename.set
            (boost::filesystem::path(path).filename().string(), df);
    }
}

boost::filesystem::path Registry::confRoot()
{
    return detail::root;
}

boost::filesystem::path defaultPath()
{
    return utility::buildsys::installPath("etc/vadstena-registry");
}

} } // namespace vadstena::registry
