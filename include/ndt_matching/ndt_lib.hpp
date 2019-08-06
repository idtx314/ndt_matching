#ifndef NDT_MATCHING__NDT_LIB_HPP_
#define NDT_MATCHING__NDT_LIB_HPP_


#include "ndt_matching/visibility_control.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <eigen3/Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose.hpp>

namespace ndt_matching
{

class NdtLib
{
public:
  NdtLib();
  virtual ~NdtLib();

  int update_map(const std_msgs::msg::String::SharedPtr msg);
  auto align_scan(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

private:
  class Cell
  {
  public:
    Cell();

    std::vector<Eigen::MatrixXd> point_list_;
    Eigen::MatrixXd mean_vector_;
    Eigen::MatrixXd covariance_matrix_;

    void initialize();
    bool is_initialized();

  private:
    bool initialized_;

    void equation_2();
    void equation_3();
  };


  double equation_4(Eigen::MatrixXd input_point, Eigen::MatrixXd q, Eigen::MatrixXd C);
  // double equation_6();
  // double equation_7();
  // double equation_8();
  // double equation_13();
  // double equation_17();
  // double equation_18();
  // double subcalculations();

};




}  // namespace ndt_matching

#endif  // NDT_MATCHING__NDT_LIB_HPP_
