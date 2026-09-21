#include <gtest/gtest.h>
#include <limits>
#include "home_robotics_manipulation/gripper_interface.hpp"
#include "home_robotics_manipulation/cube_demo_policy.hpp"
namespace hm=home_robotics_manipulation;
TEST(CubeDemo, GripperInterfaceAndWidthLimits) {
  EXPECT_STREQ(hm::GripperInterface::endpoint,"/panda1_gripper_controller/gripper_cmd");
  EXPECT_DOUBLE_EQ(hm::GripperInterface::fingerPosition(0.064),0.032);
  EXPECT_THROW(hm::GripperInterface::fingerPosition(0.081),std::runtime_error);
  EXPECT_THROW(hm::GripperInterface::fingerPosition(-0.01),std::runtime_error);
  EXPECT_THROW(hm::GripperInterface::fingerPosition(std::numeric_limits<double>::quiet_NaN()),std::runtime_error);
}
TEST(CubeDemo, DefaultNoExecutionAndEverySafetyGateRequired) {
  hm::DemoReadiness ready{true,true,true,true,true,true,true,true,true,0.8};
  EXPECT_FALSE(hm::physicalCommandsAllowed(false,ready));EXPECT_TRUE(hm::physicalCommandsAllowed(true,ready));
  for(auto member:{&hm::DemoReadiness::joints_fresh,&hm::DemoReadiness::cube_fresh,&hm::DemoReadiness::scene_synced,
      &hm::DemoReadiness::arm_active,&hm::DemoReadiness::gripper_active,&hm::DemoReadiness::static_loaded,
      &hm::DemoReadiness::five_objects,&hm::DemoReadiness::cube_world_unique,&hm::DemoReadiness::full_state_valid}){
    auto missing=ready;missing.*member=false;EXPECT_FALSE(hm::physicalCommandsAllowed(true,missing));}
  ready.rail_separation=0.699;EXPECT_FALSE(ready.ready());
  ready.rail_separation=std::numeric_limits<double>::quiet_NaN();EXPECT_FALSE(ready.ready());
}
TEST(CubeDemo, LiftBounds) {
  for(double x:{0.08,0.10,0.15})EXPECT_TRUE(hm::validLiftHeight(x));
  for(double x:{-1.0,0.0,0.079,0.151,std::numeric_limits<double>::quiet_NaN()})EXPECT_FALSE(hm::validLiftHeight(x));
}
TEST(CubeDemo, AtomicUniqueWorldAttachedWorldLifecycle) {
  moveit_msgs::msg::AttachedCollisionObject a;a.object.id="cube";a.link_name="panda1_hand";
  EXPECT_THROW(hm::attachCube(a,false,true),std::runtime_error);
  EXPECT_THROW(hm::attachCube(a,true,false),std::runtime_error);
  auto attached=hm::attachCube(a,true,true);
  EXPECT_TRUE(attached.world.collision_objects.empty());ASSERT_EQ(attached.robot_state.attached_collision_objects.size(),1u);

  EXPECT_EQ(attached.robot_state.attached_collision_objects[0].object.operation,moveit_msgs::msg::CollisionObject::ADD);
  moveit_msgs::msg::CollisionObject world;world.id="cube";
  EXPECT_THROW(hm::releaseCube(world,false,false),std::runtime_error);
  EXPECT_THROW(hm::releaseCube(world,true,true),std::runtime_error);
  auto released=hm::releaseCube(world,true,false);
  ASSERT_EQ(released.world.collision_objects.size(),1u);ASSERT_EQ(released.robot_state.attached_collision_objects.size(),1u);
  EXPECT_EQ(released.robot_state.attached_collision_objects[0].object.operation,moveit_msgs::msg::CollisionObject::REMOVE);
  EXPECT_EQ(released.world.collision_objects[0].operation,moveit_msgs::msg::CollisionObject::ADD);
}
