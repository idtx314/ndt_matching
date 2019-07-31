#ifndef NDT_MATCHING__NDT_LIB_HPP_
#define NDT_MATCHING__NDT_LIB_HPP_

#include <eigen3/Eigen/Dense>
#include "ndt_matching/visibility_control.h"

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

  int update_map(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  auto align_scan(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

private:
  Eigen::MatrixXd equation_2(const sensor_msgs::msg::PointCloud2::SharedPtr input);
  double equation_3();
  double equation_4();
  // double equation_6();
  // double equation_7();
  // double equation_8();
  // double equation_13();
  // double equation_17();
  // double equation_18();

};




}  // namespace ndt_matching

#endif  // NDT_MATCHING__NDT_LIB_HPP_
