#include "ndt_matching/ndt_lib.hpp"

// Debug
#include <iostream>

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

    // Safely attempt file read
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    // if (pcl::io::loadPCDFile<pcl::PointXYZ> ("map.pcd", *cloud) == -1)
    // {
    //     PCL_ERROR ("Couldn't read file\n");
    //     return -1;
    // }
    // std::cout << "Loaded "
    //         << cloud->width * cloud->height
    //         << " data points from test pcd with the following fields: "
    //         << std::endl;

    // // Get the install/<package>/share/<package> directory of the current package. Can throw PackageNotFoundError exception
    // std::string path = ament_index_cpp::get_package_share_directory("ndt_matching");
    // std::cout << path << std::endl;

    // // PointCloud 2 Translation Testing
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::fromROSMsg(*msg, *cloud);
    // sensor_msgs::msg::PointCloud2::SharedPtr msg_out;
    // msg_out = std::make_shared<sensor_msgs::msg::PointCloud2>();
    // pcl::toROSMsg(*cloud,*msg_out);


    return 0;
}

auto NdtLib::align_scan(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    auto input = msg;

    // TODO: This function should perform alignment between the point cloud passed to it and the map stored in the NdtLib object.
    geometry_msgs::msg::Pose::SharedPtr msg_out = std::make_shared<geometry_msgs::msg::Pose>();

    // Debug calls
    equation_2(msg);

    return msg_out;
}

Eigen::MatrixXd NdtLib::equation_2(const sensor_msgs::msg::PointCloud2::SharedPtr ref_points)
{
    /* TODO
    Equation 2. Calculates mean vector of a set of points.
    Inputs:
        a pointcloud containing the points in a map cell.
    Outputs:
        a 3x1matrix q, representing the mean vector of the point cloud.
    */

    Eigen::MatrixXd q(3,1);
    q(0,0)=0;
    q(1,0)=0;
    q(2,0)=0;

    // Convert ref_points to PCL_cloud

    // Cycle through points in PCL_cloud
        // Make point into a 3vector
        // q = q + 3vector
    // q = q / number_of_points



    return q;
}

Eigen::MatrixXd NdtLib::equation_3(const sensor_msgs::msg::PointCloud2::SharedPtr ref_points, Eigen::MatrixXd mean_vec)
{
    /* TODO
    Equation 3. Calculates the covariance matrix of a set of points.
    Inputs:
        a pointcloud containing the points in a map cell.
        a 3x1 matrix representing the mean vector of the point cloud.
    Outputs:
        a 3x3 matrix C, representing the covariance of the point cloud.
    */


    Eigen::MatrixXd C(3,3);
    // Initialize elements to 0

    // Convert ref_points to PCL_cloud
    // Cycle through points in PCL_cloud
        // Make point into a 3vector
        // C = C + ((3vector - mean_vec)*(3vector-mean_vec).transpose())
    // C = C / (number_of_points - 1)

    return C;
}

double NdtLib::equation_4(Eigen::MatrixXd input_point, Eigen::MatrixXd q, Eigen::MatrixXd C)
{
    /*
    Equation 4. Calculates the probability that a given point would be present based on the normal distribution representing the points in this cell.
    Inputs:
        a 3x1 matrix representing a point of interest.
        a 3x1 matrix represeting the mean vector of the reference points in this cell.
        a 3x3 matrix representing the covariance of the reference points in this cell.
    Outputs:
        p, a double representing the probability of a point being present at the point of interest, from 0 to 1.
    */

    double p;
    double c = 1;  // Normalizing constant

    // Calculate probability
    Eigen::MatrixXd temp = ( -((input_point - q).transpose()*C*(input_point-q))/2.0 );
    p = 1.0/c * std::exp(temp(0,0));

    return p;
}



// TODO
// double NdtLib::equation_6()
    // Depends on equation 4 and equation 13
// double NdtLib::equation_7()
    // Depends on equation 2 and equation 13
// double NdtLib::equation_8()
    // Depends on equation 17, equation 7, and equation 3
// double NdtLib::equation_13()
    // Independent
// double NdtLib::equation_17()
    // Depends on subcalculations
// double NdtLib::equation_18()
    // Depends on subcalculations
// double NdtLib::subcalculations()




}  // namespace ndt_matching
