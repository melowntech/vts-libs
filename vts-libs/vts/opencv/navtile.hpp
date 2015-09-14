#ifndef vadstena_libs_vts_opencv_navtile_hpp
#define vadstena_libs_vts_opencv_navtile_hpp

#include <opencv2/core/core.hpp>

#include "../navtile.hpp"

namespace vadstena { namespace vts { namespace opencv {

class NavTile : public vts::NavTile {
public:
    NavTile() {}

    virtual void serialize(std::ostream &os) const;

    virtual void deserialize(const HeightRange &heightRange, std::istream &is
                             , const boost::filesystem::path &path);

    typedef cv::Mat Data;

    const Data& data() const { return data_; }

    void set(const Data &data);

private:
    Data data_;
};

} } } // namespace vadstena::vts::opencv

#endif // vadstena_libs_vts_opencv_navtile_hpp
