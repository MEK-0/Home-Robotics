#include <gtest/gtest.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "home_robotics_manipulation/placement_target.hpp"
#include "home_robotics_manipulation/object_registry.hpp"
#include "home_robotics_manipulation/cube_demo_policy.hpp"
namespace hm=home_robotics_manipulation;
std::string config(){return ament_index_cpp::get_package_share_directory("home_robotics_control")+"/config/";}
TEST(Placement, NamedLookupTopAndObjectHeight) {
  auto t=hm::PlacementTarget::lookup(config()+"scene.yaml","surface_left_2");
  auto s=YAML::LoadFile(config()+"scene.yaml")["scene"]["surfaces"][t.target_id];
  EXPECT_DOUBLE_EQ(t.surface_top,s["pose"]["position"][2].as<double>());
  EXPECT_DOUBLE_EQ(t.surface_top,s["top_height"].as<double>());
  hm::ObjectRegistry r(config()+"objects.yaml");geometry_msgs::msg::Quaternion q;q.w=1;
  auto p=t.objectPose(r.at("cube"),q);
  EXPECT_DOUBLE_EQ(p.position.z,t.surface_top+0.025+t.clearance);
  EXPECT_DOUBLE_EQ(p.position.x,s["pose"]["position"][0].as<double>());
  EXPECT_TRUE(t.contains(r.at("cube"),p));
  auto above=t.prePlace(p);EXPECT_NEAR(above.position.z-p.position.z,0.1,1e-12);
  EXPECT_EQ(above.orientation,p.orientation);EXPECT_EQ(above.position.x,p.position.x);
  p.position.x+=t.size.x()/2;EXPECT_FALSE(t.contains(r.at("cube"),p));
  EXPECT_THROW(hm::PlacementTarget::lookup(config()+"scene.yaml","missing"),std::runtime_error);
  EXPECT_THROW(hm::PlacementTarget::lookup(config()+"scene.yaml",""),std::runtime_error);
  EXPECT_THROW(hm::PlacementTarget::lookup(config()+"scene.yaml","surface_right_1"),std::runtime_error);
}
TEST(Placement, OrientationChangesBoundingHeight) {
  auto t=hm::PlacementTarget::lookup(config()+"scene.yaml","surface_left_2");
  hm::ObjectRegistry r(config()+"objects.yaml");geometry_msgs::msg::Quaternion q;
  q.w=std::cos(M_PI/8);q.x=std::sin(M_PI/8);
  EXPECT_NEAR(t.objectPose(r.at("cube"),q).position.z,t.surface_top+std::sqrt(2)*0.025+t.clearance,1e-12);
}
TEST(Placement, VerifiedGraspRequiredBeforeTransportAndDropFails) {
  using S=hm::ManipulationState;
  for(auto s:{S::IDLE,S::OBJECT_SELECTED,S::GRIPPER_CLOSING,S::GRASP_VERIFYING,S::GRASP_VERIFIED})
    EXPECT_FALSE(hm::transition_allowed(s,S::TRANSPORTING));
  EXPECT_TRUE(hm::transition_allowed(S::LIFTING,S::TRANSPORTING));
  EXPECT_TRUE(hm::transition_allowed(S::TRANSPORTING,S::FAILED));
  EXPECT_FALSE(hm::transition_allowed(S::FAILED,S::PLACING));
}

#include <moveit/planning_scene/planning_scene.hpp>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
TEST(Placement, ActualPlanningSceneOwnershipRoundTrip) {
  auto urdf=urdf::parseURDF("<robot name='fixture'><link name='world'/><link name='panda1_hand'/><joint name='mount' type='fixed'><parent link='world'/><child link='panda1_hand'/></joint></robot>");
  auto semantic=std::make_shared<srdf::Model>();
  ASSERT_TRUE(semantic->initString(*urdf,"<robot name='fixture'/>"));
  auto model=std::make_shared<moveit::core::RobotModel>(urdf,semantic);
  planning_scene::PlanningScene scene(model);
  hm::ObjectRegistry registry(config()+"objects.yaml");
  geometry_msgs::msg::PoseStamped pose;pose.header.frame_id="world";pose.pose.orientation.w=1;pose.pose.position.z=0.7;
  auto world=hm::world_object(registry.at("cube"),pose);
  ASSERT_TRUE(scene.processCollisionObjectMsg(world));
  moveit_msgs::msg::AttachedCollisionObject object;object.link_name="panda1_hand";object.object=world;
  scene.setPlanningSceneDiffMsg(hm::attachCube(object,true,true));
  EXPECT_FALSE(scene.getWorld()->hasObject("cube"));
  ASSERT_TRUE(scene.getCurrentState().hasAttachedBody("cube"));
  EXPECT_THROW(hm::releaseCube(world,false,false),std::runtime_error);
  EXPECT_TRUE(scene.getCurrentState().hasAttachedBody("cube"));
  scene.setPlanningSceneDiffMsg(hm::releaseCube(world,true,false));
  EXPECT_TRUE(scene.getWorld()->hasObject("cube"));
  EXPECT_FALSE(scene.getCurrentState().hasAttachedBody("cube"));
  EXPECT_EQ(scene.getWorld()->getObjectIds().size(),1u);
}

TEST(Placement, FailureClassificationAndPlaceRetention) {
  using S=hm::ManipulationState;
  EXPECT_EQ(hm::failure_code("IK_FAILED descent",S::PLACING),"PLACE_FAILED");
  EXPECT_EQ(hm::failure_code("OBJECT_DROPPED drift",S::TRANSPORTING),"OBJECT_DROPPED");
  EXPECT_EQ(hm::failure_code("SCENE_SYNC_FAILED timeout",S::PLACING),"SCENE_SYNC_FAILED");
  EXPECT_EQ(hm::failure_code("NO_CONTACT",S::GRASP_VERIFYING),"NO_CONTACT");
}

TEST(Placement, UnsupportedReleaseDoesNotCreateSceneMutation) {
  moveit_msgs::msg::CollisionObject world;world.id="cube";
  EXPECT_THROW(hm::releaseCube(world,false,false),std::runtime_error);
  EXPECT_THROW(hm::releaseCube(world,true,true),std::runtime_error);
  moveit_msgs::msg::AttachedCollisionObject object;object.object.id="cube";object.link_name="panda1_hand";
  EXPECT_THROW(hm::attachCube(object,false,true),std::runtime_error);
}
