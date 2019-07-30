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

    // TODO: This function should perform alignment between the point cloud passed to it and the map stored in the NdtLib object.
    auto msg_out = std::make_shared<geometry_msgs::msg::Pose>();
    return msg_out;
}


}  // namespace ndt_matching
