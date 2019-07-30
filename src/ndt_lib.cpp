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

geometry_msgs::msg::Pose NdtLib::align_scan(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    geometry_msgs::msg::Pose msg_out;
    return msg_out;
}

int re_3()
{
    return 3;
}

}  // namespace ndt_matching
