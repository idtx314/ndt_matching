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

    return msg_out;
}

double equation_2()
{
    /* TODO
    Equation 2. Calculates mean vector of a set of points.
    Inputs:
        a pointcloud containing the points in a map cell.
    Outputs:
        a 3 vector q, representing the mean vector of the point cloud.
    */

    return 0;
}

double equation_3()
{
    /* TODO
    Equation 3. Calculates the covariance matrix of a set of points.
    Inputs:
        a pointcloud containing the points in a map cell.
        a 3 vector representing the mean vector of the point cloud.
    Outputs:
        a 3x3 matrix C, representing the covariance of the point cloud.
    */

    return 0;
}

double equation_4()
{
    /* TODO
    Equation 4. Calculates the probability that a given point would be present based on the normal distribution representing the points in this cell.
    Inputs:
        a 3 vector representing a point of interest.
        a 3 vector represeting the mean vector of the reference points in this cell.
        a 3x3 matrix representing the covariance of the reference points in this cell.
    Outputs:
        p, the probability of a point being present at the point of interest from 0 to 1.
    */

    return 0;
}



// TODO
// double equation_6()
    // Depends on equation 4 and equation 13
// double equation_7()
    // Depends on equation 2 and equation 13
// double equation_8()
    // Depends on equation 17, equation 7, and equation 3
// double equation_13()
    // Independent
// double equation_17()
    // Depends on Jacobian subcalculations
// double equation_18()
    // Depends on H7 subcalculations




}  // namespace ndt_matching
