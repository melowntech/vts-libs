#include "../registry.hpp"

namespace vadstena { namespace registry {

namespace detail {
    Srs::dict srs;

    ReferenceFrame::dict referenceFrames;

    BoundLayer::dict boundLayers;
    BoundLayer::ndict nBoundLayers;

    Credit::dict credits;
    Credit::ndict nCredits;
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

void Registry::init(const boost::filesystem::path &confRoot)
{
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
}

} } // namespace vadstena::registry
