#ifndef NDT_MATCHING__NDT_LIB_HPP_
#define NDT_MATCHING__NDT_LIB_HPP_


#include "ndt_matching/visibility_control.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <eigen3/Eigen/Dense>
#include <pcl/io/pcd_io.h> // TODO: change from pcl to ros_pcl
#include <pcl/point_types.h>
#include <pcl/common/common.h>
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

    std::vector<Eigen::Matrix<float,3,1>> point_list_;
    Eigen::Matrix<float,3,1> mean_vector_;
    Eigen::Matrix<float,3,3> covariance_matrix_;

    void initialize();
    bool is_initialized();

  private:
    bool initialized_;

    void equation_2();
    void equation_3();
  };

  std::vector<NdtLib::Cell> cell_list_;
  std::vector<int> test_vec_;
  Eigen::Matrix<float,4,1> lower_bound_, upper_bound_;

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
