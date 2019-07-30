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

int NdtLib::update_map(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    //TODO: This function should take in a point cloud, compose an appropriately sized vector of cell objects, sort points into the relevant cell objects, and store the vector, the segmenting cell size, and the dimensions of the space as metadata in the parent class.
    auto input = msg;
    return 0;
}

auto NdtLib::align_scan(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    auto input = msg;

    // TODO: This function should perform alignment between the point cloud passed to it and the map stored in the NdtLib object.
    geometry_msgs::msg::Pose::SharedPtr msg_out = std::make_shared<geometry_msgs::msg::Pose>();
    msg_out->position.x = cell_center();

    return msg_out;
}


double NdtLib::cell_center()
{
    return 3.5;
}




}  // namespace ndt_matching
