#include "ndt_matching/ndt_lib.hpp"

namespace ndt_matching
{

NdtLib::NdtLib()
{
    // Hello World!
}

NdtLib::~NdtLib()
{
}

int NdtLib::map_update()
{
    return 0;
}

geometry_msgs::msg::Pose::SharedPtr NdtLib::align_scan(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    auto msg_out = std::make_shared<geometry_msgs::msg::Pose>();
    return msg_out;
}

int re_3()
{
    return 3;
}

}  // namespace ndt_matching
