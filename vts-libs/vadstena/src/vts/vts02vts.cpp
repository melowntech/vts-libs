#include <cstdlib>
#include <string>
#include <iostream>
#include <algorithm>

#include <jpeglib.h>

#include "dbglog/dbglog.hpp"
#include "utility/streams.hpp"

#include "utility/buildsys.hpp"
#include "utility/gccversion.hpp"
#include "utility/progress.hpp"
#include "utility/streams.hpp"
#include "utility/openmp.hpp"

#include "geo/csconvertor.hpp"

#include "vadstena-libs/utility.hpp"

#include "vadstena-libs/vts0.hpp"
#include "vadstena-libs/vts0/tileset-advanced.hpp"

#include "vadstena-libs/vts.hpp"
#include "vadstena-libs/vts/encoder.hpp"

#include "vadstena-libs/binmesh.hpp"

namespace po = boost::program_options;
namespace vs = vadstena::storage;
namespace vr = vadstena::registry;
namespace vts0 = vadstena::vts0;
namespace vts = vadstena::vts;
namespace va = vadstena;
namespace fs = boost::filesystem;

namespace {

math::Size2 jpegSize(std::istream &is, const fs::path &path)
{
    char buf[1024];
    std::size_t size(sizeof(buf));

    {
        auto exc(utility::scopedStreamExceptions(is));

        // clear EOF bit
        is.exceptions(exc.state()
                      & ~(std::ios_base::failbit | std::ios_base::eofbit));
        size = is.read(buf, size).gcount();
        is.clear();
    }

    ::jpeg_decompress_struct cinfo;
    ::jpeg_error_mgr jerr;
    cinfo.err = ::jpeg_std_error(&jerr);
    ::jpeg_create_decompress(&cinfo);

    ::jpeg_mem_src(&cinfo, reinterpret_cast<unsigned char*>(buf), size);
    auto res(::jpeg_read_header(&cinfo, TRUE));

    if (res != JPEG_HEADER_OK) {
        LOGTHROW(err4, std::runtime_error)
            << "Unable to determine size of JPEG " << path << ".";
    }

    // fine
    return math::Size2(cinfo.image_width, cinfo.image_height);
}

struct Config {
    boost::optional<std::uint16_t> textureLayer;
};

class Vts02Vts : public va::UtilityBase
{
public:
    Vts02Vts()
        : UtilityBase("vts02vts", BUILD_TARGET_VERSION)
        , createMode_(vts::CreateMode::failIfExists)
    {
    }

private:
    virtual void configuration(po::options_description &cmdline
                               , po::options_description &config
                               , po::positional_options_description &pd)
        UTILITY_OVERRIDE;

    virtual void configure(const po::variables_map &vars)
        UTILITY_OVERRIDE;

    virtual bool help(std::ostream &out, const std::string &what) const
        UTILITY_OVERRIDE;

    virtual int run() UTILITY_OVERRIDE;

    fs::path input_;
    fs::path output_;

    vts::CreateMode createMode_;

    vts::StaticProperties properties_;

    Config config_;
};

void Vts02Vts::configuration(po::options_description &cmdline
                             , po::options_description &config
                             , po::positional_options_description &pd)
{
    cmdline.add_options()
        ("input", po::value(&input_)->required()
         , "Path to input (vts0) tile set.")
        ("output", po::value(&output_)->required()
         , "Path to output (vts) tile set.")
        ("overwrite", "Existing tile set gets overwritten if set.")

        ("referenceFrame", po::value(&properties_.referenceFrame)->required()
         , "Reference frame.")

        ("textureLayer", po::value<std::string>()
         , "String/numeric id of bound layer to be used as external texture "
         "in generated meshes. Turns on generation of external texture "
         "coordinates if set.")
        ;

    registryConfiguration(cmdline);

    pd.add("input", 1);
    pd.add("output", 1);

    (void) config;
}

void Vts02Vts::configure(const po::variables_map &vars)
{
    registryConfigure(vars);

    createMode_ = (vars.count("overwrite")
                   ? vts::CreateMode::overwrite
                   : vts::CreateMode::failIfExists);

    if (vars.count("textureLayer")) {
        auto value(vars["textureLayer"].as<std::string>());

        vr::BoundLayer layer;
        try {
            layer = vr::Registry::boundLayer(boost::lexical_cast<int>(value));
        } catch (boost::bad_lexical_cast) {
            layer = vr::Registry::boundLayer(value);
        }

        if (layer.type != vr::BoundLayer::Type::raster) {
            throw po::validation_error
                (po::validation_error::invalid_option_value, "textureLayer");
        }
        config_.textureLayer = layer.numericId;
    }
}

bool Vts02Vts::help(std::ostream &out, const std::string &what) const
{
    if (what.empty()) {
        out << R"RAW(vts02vts
usage
    vts02vts INPUT OUTPUT [OPTIONS]

)RAW";
    }
    return false;
}

class Encoder : public vts::Encoder {
public:
    Encoder(const boost::filesystem::path &path
            , const vts::StaticProperties &properties, vts::CreateMode mode
            , const vts0::TileSet::pointer &input
            , const Config &config)
        : vts::Encoder(path, properties, mode)
        , config_(config)
        , input_(input), aa_(input_->advancedApi())
        , ti_(aa_.tileIndex()), cti_(ti_)
    {
        cti_.makeFull().makeComplete();
        // set constraints: from zero to max LOD
        setConstraints(Constraints()
                       .setLodRange
                       (vs::LodRange(0, input->lodRange().max)));
    }

private:
    virtual TileResult
    generate(const vts::TileId &tileId
             , const vr::ReferenceFrame::Division::Node &node
             , const math::Extents2 &divisionExtents) UTILITY_OVERRIDE;

    virtual void finish(vts::TileSet&) UTILITY_OVERRIDE {}

    const Config config_;

    vts0::TileSet::pointer input_;
    vts0::TileSet::AdvancedApi aa_;
    const vts0::TileIndex &ti_;
    vts0::TileIndex cti_;
};

vts0::Mesh loadMesh(const vs::IStream::pointer &is)
{
    auto mesh(va::loadBinaryMesh(is->get()));
    is->close();
    return mesh;
}

class Atlas : public vts::Atlas {
public:
    Atlas(const vs::IStream::pointer &stream)
        : stream_(stream), area_(math::area(jpegSize(*stream, stream->name())))
    {}

private:
    virtual std::size_t size() const UTILITY_OVERRIDE { return 1; }

    virtual vts::multifile::Table serialize_impl(std::ostream &os) const
        UTILITY_OVERRIDE
    {
        stream_->get().seekg(0);
        auto start(os.tellp());
        os << stream_->get().rdbuf();
        auto end(os.tellp());

        vts::multifile::Table table;
        table.entries.emplace_back(start, end - start);
        return table;
    }

    virtual void deserialize_impl(std::istream&, const vts::multifile::Table&)
        UTILITY_OVERRIDE
    {
        LOGTHROW(err4, std::runtime_error)
            << "This atlas is serialize-only.";
    }

    virtual std::size_t area(std::size_t index) const {
        UTILITY_OVERRIDE
        if (index) {
            LOGTHROW(err4, std::runtime_error)
                << "This atlas has just one image+.";
        }

        return area_;
    }

    vs::IStream::pointer stream_;
    std::size_t area_;
};

inline vts0::TileId asVts(const vts::TileId tileId) {
    return vts0::TileId(tileId.lod, tileId.x, tileId.y);
}

class TextureNormalizer {
public:
    TextureNormalizer(const math::Extents2 &divisionExtents)
        : size_(size(divisionExtents))
        , origin_(divisionExtents.ll)
    {}

    math::Point2 operator()(const math::Point3 &p) const {
        // NB: origin is in the upper-left corner
        return { (p(0) - origin_(0)) / size_.width
                , 1.0 - ((p(1) - origin_(1)) / size_.height) };
    };

private:
    math::Size2f size_;
    math::Point2 origin_;
};

vts::Mesh::pointer createMesh(const vts0::Mesh &gm
                              , const geo::SrsDefinition &srcSrs
                              , const vr::Srs &dstSrs
                              , const math::Extents2 &divisionExtents
                              , bool externalTextureCoordinates
                              , boost::optional<std::uint16_t> textureLayer)
{
    // just one submesh
    auto mesh(std::make_shared<vts::Mesh>());
    mesh->submeshes.emplace_back();
    auto &sm(mesh->submeshes.back());

    // copy vertices
    geo::CsConvertor conv(srcSrs, dstSrs.srsDef);
    TextureNormalizer tn(divisionExtents);
    for (const auto &v : gm.vertices) {
        // convert v from division SRS to physical SRS (can be no-op)
        sm.vertices.push_back(conv(v));

        // generate external texture coordinates if instructed
        if (externalTextureCoordinates) { sm.etc.push_back(tn(v)); }
    }

    if (externalTextureCoordinates) {
        sm.textureLayer = textureLayer;
    }

    // copy texture coordinates
    std::transform(gm.texcoords.begin(), gm.texcoords.end()
                   , std::back_inserter(sm.tc)
                   , [&](const math::Point3 &p)
    {
        return math::Point2(p(0), p(1));
    });

    for (const auto &f : gm.facets) {
        sm.faces.emplace_back(f.v[0], f.v[1], f.v[2]);
        sm.facesTc.emplace_back(f.t[0], f.t[1], f.t[2]);
    }

    // TODO: generate coverage mask
    // // set coverage mask
    // mesh->coverageMask = coverageMask;

    return mesh;
}

Encoder::TileResult
Encoder::generate(const vts::TileId &tileId
                  , const vr::ReferenceFrame::Division::Node &node
                  , const math::Extents2 &divisionExtents)
{
    const auto nodeSrs(vr::Registry::srs(node.srs));
    geo::SrsDefinition spatialDivisionSrs(nodeSrs.srsDef);

    auto vts0Id(asVts(tileId));

    if (!cti_.exists(vts0Id)) {
        // neither this nor any child tile exists -> no data
        return TileResult::Result::noData;
    }

    if (!ti_.exists(vts0Id)) {
        // this tile doesn't exist but there are some children
        return TileResult::Result::noDataYet;
    }

    // load mesh; NB: mesh is in space division srs, just convert to physical
    // and here we go
    vts0::Mesh mesh;
    vs::IStream::pointer atlasStream;

    UTILITY_OMP(critical)
    {
        // NB: access to tileset is not thread safe!
        mesh = loadMesh(aa_.input(vts0Id, vs::TileFile::mesh));
        atlasStream = aa_.input(vts0Id, vs::TileFile::atlas);
    }

    vts::Encoder::TileResult result(TileResult::Result::data);
    auto &tile(result.tile);

    tile.atlas = std::make_shared<Atlas>(atlasStream);

    // TODO: convert mesh
    tile.mesh = createMesh(mesh, spatialDivisionSrs, physicalSrs()
                           , divisionExtents, bool(node.boundLayerLod)
                           , config_.textureLayer);

    return result;
}

int Vts02Vts::run()
{
    // open vts0 tileset
    auto input(vts0::openTileSet(input_));

    properties_.id = input->getProperties().id;

    // TODO: make checks here

    // run the encoder
    Encoder(output_, properties_, createMode_, input, config_).run();

    // all done
    LOG(info4) << "All done.";
    return EXIT_SUCCESS;
}

} // namespace

int main(int argc, char *argv[])
{
    return Vts02Vts()(argc, argv);
}
