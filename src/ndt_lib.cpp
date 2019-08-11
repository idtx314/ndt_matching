#include "ndt_matching/ndt_lib.hpp"

// Debug
#include <iostream>

namespace ndt_matching
{

NdtLib::NdtLib()
{
}

NdtLib::~NdtLib()
{
}

int NdtLib::update_map(const std_msgs::msg::String::SharedPtr msg)
{
    /** This function loads a pcd file and segments the resulting point cloud data to be used as a reference map for the NDT Algorithm.

    This function accepts a string containing the absolute path to a pcd input file.
    The contents of the file will be segmented into cubic meter cells and the metadata of each cell calculated and saved into the parent NdtLib object
    Returns an int indicating success (0) or error (1).
    */

    //TODO: This function should take in a string, use it to find and read a PCD file into a pcl pointcloud, save the dimensions and offset of the cloud as metadata in the parent class, compose an appropriately sized vector of cell objects, sort points into the relevant cell objects, then calculate metadata for each cell.
    // Points will be allocated to cells based on physical location. This will produce float representation errors when dealing with outlying points, but as long as each point is sorted into exactly one cell the effect of this should be minor.

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




    // Begin comment skeleton

    // Read .pcd file into a pcl point cloud to use as a new reference map.
    pcl::PointCloud<pcl::PointXYZ> ref_cloud;
    if(pcl::io::loadPCDFile<pcl::PointXYZ>(msg->data, ref_cloud) == -1)
        {return 1;}

    // Get bounding point value on each axis of the reference cloud.
    pcl::getMinMax3D(ref_cloud, lower_bound_, upper_bound_);
    // Round away from center of cloud
    upper_bound_ << std::ceil(upper_bound_(0,0)),
                    std::ceil(upper_bound_(1,0)),
                    std::ceil(upper_bound_(2,0)),
                    upper_bound_(3,0);
    lower_bound_ << std::floor(lower_bound_(0,0)),
                    std::floor(lower_bound_(1,0)),
                    std::floor(lower_bound_(2,0)),
                    lower_bound_(3,0);

    // Clear and set the parent object cell vector to the correct size.
    double x_range = upper_bound_(0,0) - lower_bound_(0,0);
    double y_range = upper_bound_(1,0) - lower_bound_(1,0);
    double z_range = upper_bound_(2,0) - lower_bound_(2,0);
    int vector_size = static_cast<int>(x_range*y_range*z_range);
    cell_list_ = std::vector<NdtLib::Cell> (vector_size);

    // For each point in ref_cloud determine the index of the appropriate cell in cell_list_, convert to an eigen matrix, and add the point to point_list_ in that cell

    double segment_size = 1;
    int width_x = static_cast<int>(x_range);
    int width_y = static_cast<int>(y_range);

    for (pcl::PointXYZ point : ref_cloud)
    {
        // Convert to eigen matrix
        Eigen::Matrix<float,3,1> e_point;
        e_point << point.x, point.y, point.z;

        // Get Index
        Eigen::Matrix<float,3,1> i_point;
        i_point = e_point;
            //shift points into quadrant 1
        i_point(0,0) = i_point(0,0) - lower_bound_(0,0);
        i_point(1,0) = i_point(1,0) - lower_bound_(1,0);
        i_point(2,0) = i_point(2,0) - lower_bound_(2,0);
            //divide by segment size
        i_point = i_point / segment_size;
            //calculate index
        int index = static_cast<int>(i_point(0,0)) +
                    width_x * static_cast<int>(i_point(1,0)) +
                    width_x * width_y * static_cast<int>(i_point(2,0));

        // Add point to correct cell
        // cell_list_[index].point_list_.push_back(e_point);

        break;
    }


// index =
    // int( shifted_xcoordinate/segment_size ) +
    // x_width * int(shifted_ycoordinate/segment_size) +
    // x_width * y_width * int(shifted_zcoordinate/segment_size)





    // For each cell in cell_list_ trigger initialization




    // Return Success or Failure.
    return 0;
}

NdtLib::Cell::Cell()
{
    initialized_ = false;
}

void NdtLib::Cell::initialize()
{
    /** This function calculates the mean vector q and the covariance matrix C of the points currently included in the Cell object and sets the relevant member variables. It then marks the cell as initialized.
    */

    // TODO






    initialized_ = true;
}

bool NdtLib::Cell::is_initialized()
{
    return initialized_;
}



auto NdtLib::align_scan(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{

    /** This function attempts to align an input point cloud with the current reference map.

    This function accepts a PointCloud2 sensor message and aligns it with the current reference map using a 3D NDT algorithm.

    The PointCloud will be parsed using PCL and Eigen and iterated until the change in estimated pose between iterations is less than .0001 Meters/Radians. For explanation of the algorithm used, see:
        M. Magnusson, A. Lilienthal, Scan Registration for Autonomous Mining Vehicles Using 3D-NDT, 2007, Wiley Periodicals
    */

    // DEBUG: Do something with the input
    auto input = msg;

    // TODO: This function should perform alignment between the point cloud passed to it and the map stored in the NdtLib object.
    geometry_msgs::msg::Pose::SharedPtr msg_out = std::make_shared<geometry_msgs::msg::Pose>();


    return msg_out;
}

void NdtLib::Cell::equation_2()
{
    /** This function calculates the mean vector of a set of points in 3D space.

    This function uses the variable point_list_ from its parent object. It sets the 3x1 Eigen matrix variable mean_vector_ in the parent object.
    */

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


}

void NdtLib::Cell::equation_3()
{
    /** This function calculates the covariance matrix of a set of points in 3D space.

    This function uses the variables point_list and mean_vector_ from its parent object. It sets the variable covariance_matrix_ in its parent object.
    */

    /* TODO
    Equation 3. Calculates the covariance matrix of a set of points.
    Inputs:
        a pointcloud containing the points in a map cell.
        a 3x1 matrix representing the mean vector of the point cloud.
    Outputs:
        a 3x3 matrix C, representing the covariance of the point cloud.
    */


    Eigen::MatrixXd C(3,3);
    Eigen::Matrix<float,3,1> L;

    // Initialize elements to 0

    // Convert ref_points to PCL_cloud
    // Cycle through points in PCL_cloud
        // Make point into a 3vector
        // C = C + ((3vector - mean_vec)*(3vector-mean_vec).transpose())
    // C = C / (number_of_points - 1)

}

double NdtLib::equation_4(Eigen::MatrixXd input_point, Eigen::MatrixXd q, Eigen::MatrixXd C)
{
    /** This function returns the probability that a given point would be produced by a Normal Probability Distribution Function that would generate the current reference map.

    This function accepts a 3x1 matrix representing a point in space, a 3x1 matrix representing the mean vector of the reference map in that region of space, and a 3x3 matrix representing the covariance of the reference map in that region of space. It returns the probability, from 0 to 1, that an NDT function based on the reference map in that region would produce the input point.
    */

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
