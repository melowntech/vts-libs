#include <opencv2/highgui/highgui.hpp>

#include "utility/binaryio.hpp"

#include "./navtile.hpp"

namespace vadstena { namespace vts { namespace opencv {

void NavTile::serialize(const storage::OStream::pointer &os) const
{
    using utility::binaryio::write;
    std::vector<unsigned char> buf;
    cv::imencode(".jpg", image_, buf
                 , { cv::IMWRITE_JPEG_QUALITY, 75 });

    write(os->get(), buf.data(), buf.size());
    os->close();
}

void NavTile::deserialize(const storage::IStream::pointer &is)
{
    using utility::binaryio::read;
    auto& s(is->get());
    auto size(s.seekg(0, std::ios_base::end).tellg());
    s.seekg(0);
    std::vector<unsigned char> buf;
    buf.resize(size);
    read(s, buf.data(), buf.size());

    image_ = cv::imdecode(buf, CV_LOAD_IMAGE_COLOR);
    is->close();
}

} } } // namespace vadstena::vts::opencv
