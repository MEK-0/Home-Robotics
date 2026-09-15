#include <gtest/gtest.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "home_robotics_manipulation/grasp_pose_generator.hpp"
namespace hm=home_robotics_manipulation;
namespace {
hm::ObjectRegistry registry() {return hm::ObjectRegistry(ament_index_cpp::get_package_share_directory("home_robotics_control")+"/config/objects.yaml");}
// Synthetic measured-geometry fixture. Production always extracts these from RobotModel.
const hm::GripperGeometry geometry{0.10,0.1029,0.0085,0.08};
geometry_msgs::msg::Pose object_pose(){geometry_msgs::msg::Pose p;p.position.x=0.3;p.position.y=0.4;p.position.z=0.7;p.orientation.w=1;return p;}
}
TEST(Grasp, DeterministicAndDimensionDriven) {
  const auto cube=registry().at("cube");const auto p=object_pose();
  const auto a=hm::generateTopDownGrasp(cube,p,geometry), b=hm::generateTopDownGrasp(cube,p,geometry);
  EXPECT_EQ(a.pre_grasp_pose,b.pre_grasp_pose);EXPECT_EQ(a.grasp_pose,b.grasp_pose);
  EXPECT_EQ(a.object_id,"cube");EXPECT_EQ(a.robot_id,"panda1");
  EXPECT_DOUBLE_EQ(a.expected_cube_width,cube.geometry.primitives[0].dimensions[0]);
  EXPECT_DOUBLE_EQ(a.expected_gripper_width,geometry.maximum_open_width);
  EXPECT_NEAR(a.grasp_pose.position.z-p.position.z,0.008+geometry.hand_to_pad-geometry.hand_to_tcp,1e-12);
  EXPECT_NE(a.grasp_pose.position.z,p.position.z);
  EXPECT_NEAR(a.pre_grasp_pose.position.z-a.grasp_pose.position.z,0.1,1e-12);
  hm::GraspParameters parameters;parameters.pre_grasp_distance=0.12;
  const auto c=hm::generateTopDownGrasp(cube,p,geometry,parameters);
  EXPECT_NEAR(c.pre_grasp_pose.position.z-c.grasp_pose.position.z,0.12,1e-12);
  auto translated=p;translated.position.x+=0.05;
  EXPECT_NEAR(hm::generateTopDownGrasp(cube,translated,geometry).grasp_pose.position.x-a.grasp_pose.position.x,0.05,1e-12);
  EXPECT_THROW(registry().at("missing_cube"),std::out_of_range);
  EXPECT_THROW(hm::generateTopDownGrasp(registry().at("apple"),p,geometry),std::runtime_error);
}
TEST(Grasp, DownwardApproachAndCubeFaceAlignment) {
  auto p=object_pose();
  for(double yaw:{0.0,0.3,1.2}) {
    p.orientation.z=std::sin(yaw/2);p.orientation.w=std::cos(yaw/2);
    auto c=hm::generateTopDownGrasp(registry().at("cube"),p,geometry);
    const auto q=c.grasp_pose.orientation;Eigen::Quaterniond rotation(q.w,q.x,q.y,q.z);
    EXPECT_TRUE((rotation*Eigen::Vector3d::UnitZ()).isApprox(-Eigen::Vector3d::UnitZ(),1e-12));
    EXPECT_TRUE((rotation*Eigen::Vector3d::UnitY()).isApprox(Eigen::Vector3d(std::cos(yaw),std::sin(yaw),0),1e-12));
  }
}
TEST(Grasp, MinimalLocalTouchAndWorldOwnership) {
  const auto cube=registry().at("cube");auto c=hm::generateTopDownGrasp(cube,object_pose(),geometry);
  EXPECT_EQ(c.allowed_touch_links,(std::vector<std::string>{"panda1_left_finger","panda1_right_finger"}));
  collision_detection::AllowedCollisionMatrix original;
  original.setEntry("panda1_hand","cube",false);
  auto local=hm::graspEvaluationACM(original,c);
  collision_detection::AllowedCollision::Type type;
  EXPECT_TRUE(local.getAllowedCollision("panda1_left_finger","cube",type));EXPECT_EQ(type,collision_detection::AllowedCollision::ALWAYS);
  EXPECT_FALSE(original.getAllowedCollision("panda1_left_finger","cube",type));
  EXPECT_TRUE(local.getAllowedCollision("panda1_hand","cube",type));EXPECT_EQ(type,collision_detection::AllowedCollision::NEVER);
  EXPECT_FALSE(local.getAllowedCollision("panda1_link7","cube",type));
  EXPECT_EQ(cube.ownership,hm::ObjectOwnership::WORLD);
  c.allowed_touch_links.push_back("panda1_hand");EXPECT_THROW(hm::graspEvaluationACM(original,c),std::runtime_error);
}
TEST(Grasp, ExecutionJointWhitelistExcludesGrippersAndPanda2) {
  std::vector<std::string> names={"panda1_rail_joint"};for(int i=1;i<=7;++i)names.push_back("panda1_joint"+std::to_string(i));
  EXPECT_TRUE(hm::pregraspJointNamesAllowed(names));
  names.push_back("panda1_finger_joint1");EXPECT_FALSE(hm::pregraspJointNamesAllowed(names));names.pop_back();
  names[0]="panda2_rail_joint";EXPECT_FALSE(hm::pregraspJointNamesAllowed(names));
}
TEST(Grasp, RejectsUnsafeParameters) {
  hm::GraspParameters params;params.grasp_clearance=0.03;
  EXPECT_THROW(hm::generateTopDownGrasp(registry().at("cube"),object_pose(),geometry,params),std::runtime_error);
  params={};params.expected_cube_width=0.06;
  EXPECT_THROW(hm::generateTopDownGrasp(registry().at("cube"),object_pose(),geometry,params),std::runtime_error);
  params={};params.finger_open_width=0.04;
  EXPECT_THROW(hm::generateTopDownGrasp(registry().at("cube"),object_pose(),geometry,params),std::runtime_error);
  params={};params.allowed_touch_links.push_back("panda1_link7");
  EXPECT_THROW(hm::generateTopDownGrasp(registry().at("cube"),object_pose(),geometry,params),std::runtime_error);
}
