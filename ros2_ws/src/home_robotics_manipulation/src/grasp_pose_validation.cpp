#include "home_robotics_manipulation/validation_utils.hpp"

int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  auto node=rclcpp::Node::make_shared("grasp_pose_validation",rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto log=node->get_logger();
  rclcpp::executors::SingleThreadedExecutor executor; executor.add_node(node);
  std::thread spinner([&](){executor.spin();}); int result=1;
  try {
    std::string robot,object;bool execute=false,check_grasp=true,keep_alive=false;
    node->get_parameter_or<std::string>("robot",robot,"panda1");node->get_parameter_or<std::string>("object",object,"cube");
    node->get_parameter_or("execute_pregrasp",execute,false);node->get_parameter_or("validate_grasp",check_grasp,true);
    node->get_parameter_or("keep_alive",keep_alive,false);
    require(robot=="panda1" && object=="cube","Only Panda1/cube baseline supported");
    hm::GraspParameters parameters;
    node->get_parameter_or("pre_grasp_distance",parameters.pre_grasp_distance,0.10);
    node->get_parameter_or("grasp_clearance",parameters.grasp_clearance,0.008);
    node->get_parameter_or("finger_open_width",parameters.finger_open_width,0.0);
    node->get_parameter_or("expected_cube_width",parameters.expected_cube_width,0.0);
    std::vector<double> direction;node->get_parameter_or<std::vector<double>>("approach_direction",direction,{0,0,-1});
    require(direction.size()==3,"Invalid approach vector");parameters.approach_direction={direction[0],direction[1],direction[2]};
    moveit::planning_interface::MoveGroupInterface group(node,"panda1_manipulator");
    group.setEndEffectorLink("panda1_tcp");group.setPoseReferenceFrame("world");
    group.setPlanningPipelineId("ompl");group.setPlannerId("RRTConnectkConfigDefault");
    group.setPlanningTime(15);group.setMaxVelocityScalingFactor(0.1);group.setMaxAccelerationScalingFactor(0.1);
    group.setGoalPositionTolerance(0.001);group.setGoalOrientationTolerance(0.005);
    // Populate exclusively from the authoritative topic; no default/home seed and
    // no dependency on a second, lazily started current-state subscriber.
    auto state=std::make_shared<moveit::core::RobotState>(group.getRobotModel());
    auto reader=rclcpp::Node::make_shared("grasp_authoritative_reader");fresh_state(*state,reader,node);
    const auto baseline=*state;
    auto drift=std::make_shared<std::atomic<double>>(0.0);
    auto gripper_drift=std::make_shared<std::atomic<double>>(0.0);
    auto monitor=node->create_subscription<sensor_msgs::msg::JointState>("/joint_states",10,
      [baseline,drift,gripper_drift](sensor_msgs::msg::JointState::ConstSharedPtr msg){
        for(std::size_t i=0;i<msg->name.size()&&i<msg->position.size();++i){
          auto dest=msg->name[i].rfind("panda2_",0)==0?drift:
            msg->name[i].find("panda1_finger_joint")==0?gripper_drift:nullptr;
          if(!dest)continue;double delta=std::abs(msg->position[i]-baseline.getVariablePosition(msg->name[i]));
          double old=dest->load();while(delta>old&&!dest->compare_exchange_weak(old,delta)){}
        }
      });
    const auto config=ament_index_cpp::get_package_share_directory("home_robotics_control")+"/config/";
    hm::ObjectRegistry registry(config+"objects.yaml");
    auto scene_client=node->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
    auto get_scene=[&](){
      require(scene_client->wait_for_service(5s),"SCENE_SYNC_FAILED no scene service");
      auto request=std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();request->components.components=1023;
      auto future=scene_client->async_send_request(request);require(future.wait_for(5s)==std::future_status::ready,"SCENE_SYNC_FAILED scene timeout");return future.get()->scene;
    };
    auto snapshot=get_scene();planning_scene::PlanningScene scene(group.getRobotModel());scene.setPlanningSceneMsg(snapshot);
    std::set<std::string> expected;
    const auto static_config=YAML::LoadFile(config+"scene.yaml")["scene"];
    expected.insert(static_config["floor"]["id"].as<std::string>());
    for(const auto& id:{"arena_front","arena_back","arena_left","arena_right"})expected.insert(id);
    for(const auto& item:static_config["surfaces"])expected.insert(item.first.as<std::string>());
    for(const auto& item:static_config["shared_rail"]["supports"]["names"])expected.insert(item.as<std::string>());
    for(const auto& [id,metadata]:registry.objects())expected.insert(id);
    for(const auto& id:expected)require(scene.getWorld()->hasObject(id),"SCENE_SYNC_FAILED missing world object "+id);
    require(snapshot.robot_state.attached_collision_objects.empty(),"Cube and all objects must remain world objects");
    const auto acm=scene.getAllowedCollisionMatrix();
    for(const auto& link:state->getRobotModel()->getLinkModelNames()){
      collision_detection::AllowedCollision::Type type;
      require(!acm.getAllowedCollision(link,"cube",type)||type==collision_detection::AllowedCollision::NEVER,"Broad/global cube allowance found");
    }
    auto cube_pose=[&](planning_scene::PlanningScene& s){
      geometry_msgs::msg::PoseStamped msg;require(rclcpp::wait_for_message(msg,reader,"/mujoco/objects/cube/pose",5s),"SCENE_SYNC_FAILED no cube pose");
      require(msg.header.frame_id=="world","SCENE_SYNC_FAILED cube frame");
      const double age=(node->now()-rclcpp::Time(msg.header.stamp)).seconds();
      require(age>=-0.1&&age<=0.5,"SCENE_SYNC_FAILED stale cube pose");
      const auto cube=s.getWorld()->getObject("cube");require(bool(cube),"SCENE_SYNC_FAILED cube missing");
      const double error=(cube->pose_.translation()-transform(msg.pose).translation()).norm();
      const double rotation=Eigen::Quaterniond(cube->pose_.linear()).angularDistance(Eigen::Quaterniond(transform(msg.pose).linear()));
      RCLCPP_INFO(log,"Cube sync error=%.12g m orientation=%.12g rad age=%.6f s",error,rotation,age);
      geometry_msgs::msg::Pose scene_pose;
      scene_pose.position.x=cube->pose_.translation().x();scene_pose.position.y=cube->pose_.translation().y();scene_pose.position.z=cube->pose_.translation().z();
      const Eigen::Quaterniond sq(cube->pose_.linear());
      scene_pose.orientation.x=sq.x();scene_pose.orientation.y=sq.y();scene_pose.orientation.z=sq.z();scene_pose.orientation.w=sq.w();
      print_pose(log,"Cube PlanningScene",scene_pose);
      require(error<=0.002&&rotation<=1e-6,"SCENE_SYNC_FAILED cube pose discrepancy");return msg.pose;
    };
    const auto cube=cube_pose(scene);print_pose(log,"Cube",cube);
    const auto geometry=hm::pandaGripperGeometry(group.getRobotModel());
    const auto candidate=hm::generateTopDownGrasp(registry.at("cube"),cube,geometry,parameters);
    print_pose(log,"Pre-grasp",candidate.pre_grasp_pose);print_pose(log,"Grasp",candidate.grasp_pose);
    RCLCPP_INFO(log,"Geometry hand_to_tcp=%.9f hand_to_pad=%.9f pad_half_height=%.9f cube_width=%.9f open_width=%.9f pre_distance=%.9f clearance=%.9f",
      geometry.hand_to_tcp,geometry.hand_to_pad,geometry.pad_half_height,candidate.expected_cube_width,candidate.expected_gripper_width,candidate.pre_grasp_distance,parameters.grasp_clearance);
    for(const auto& name:{"panda1_finger_joint1","panda1_finger_joint2"})require(std::abs(state->getVariablePosition(name)-candidate.expected_gripper_width/2)<0.001,"Gripper must already be open; no command will be sent");
    require(valid(scene,*state,acm,log,"Current full state",true),"Current state invalid");
    auto publisher=node->create_publisher<visualization_msgs::msg::MarkerArray>("/grasp_debug",rclcpp::QoS(1).transient_local());
    const auto debug=markers(candidate);publisher->publish(debug);
    auto marker_timer=node->create_wall_timer(1s,[publisher,debug](){publisher->publish(debug);});
    const auto* jmg=state->getJointModelGroup("panda1_manipulator");
    auto solve=[&](moveit::core::RobotState& solution,const geometry_msgs::msg::Pose& target,
                   const collision_detection::AllowedCollisionMatrix& matrix,bool continuous){
      const auto seed=solution;
      auto callback=[&](moveit::core::RobotState* s,const moveit::core::JointModelGroup* g,const double* values){
        s->setJointGroupPositions(g,values);s->update();
        if(continuous)for(const auto& name:g->getVariableNames())if(std::abs(s->getVariablePosition(name)-seed.getVariablePosition(name))>0.3)return false;
        return valid(scene,*s,matrix,log,"IK candidate",false);
      };
      for(int attempt=0;attempt<12;++attempt){solution=seed;
        if(solution.setFromIK(jmg,transform(target),"panda1_tcp",0.25,callback))return true;
      }valid(scene,solution,matrix,log,"Last rejected IK state",true);return false;
    };
    auto pre=*state;require(solve(pre,candidate.pre_grasp_pose,acm,false),"IK_FAILED pre-grasp (no collision-free solution)");
    print_joints(log,"Pre-grasp IK",pre);require(valid(scene,pre,acm,log,"Pre-grasp",true),"Pre-grasp invalid");
    auto validity_client=node->create_client<moveit_msgs::srv::GetStateValidity>("/check_state_validity");
    require(validity_client->wait_for_service(5s),"Missing state validity service");
    auto vr=std::make_shared<moveit_msgs::srv::GetStateValidity::Request>();moveit::core::robotStateToRobotStateMsg(pre,vr->robot_state);
    auto vf=validity_client->async_send_request(vr);require(vf.wait_for(5s)==std::future_status::ready&&vf.get()->valid,"Server pre-grasp invalid");
    group.setStartState(*state);require(group.setPoseTarget(candidate.pre_grasp_pose,"panda1_tcp"),"Invalid pose target");
    moveit::planning_interface::MoveGroupInterface::Plan plan;const auto code=group.plan(plan);
    RCLCPP_INFO(log,"Pre-grasp plan=%s time=%.9f points=%zu",code==moveit::core::MoveItErrorCode::SUCCESS?"SUCCESS":"FAILURE",plan.planning_time,plan.trajectory.joint_trajectory.points.size());
    require(code==moveit::core::MoveItErrorCode::SUCCESS,"PLANNING_FAILED pre-grasp");
    // Refresh authoritative scene before accepting/executing the generated path.
    auto refreshed=get_scene();require(refreshed.allowed_collision_matrix==snapshot.allowed_collision_matrix,"ACM changed during validation");
    scene.setPlanningSceneMsg(refreshed);const auto latest_cube=cube_pose(scene);
    require((transform(latest_cube).translation()-transform(cube).translation()).norm()<=0.002 &&
      Eigen::Quaterniond(transform(latest_cube).linear()).angularDistance(Eigen::Quaterniond(transform(cube).linear()))<=0.01,"SCENE_SYNC_FAILED cube moved during planning");
    const auto& trajectory=plan.trajectory.joint_trajectory;
    require(!trajectory.points.empty(),"Empty plan");
    require(hm::pregraspJointNamesAllowed(trajectory.joint_names),"Trajectory must command only Panda1 arm/rail");
    auto previous=*state;std::size_t samples=0;
    for(const auto& point:trajectory.points){
      auto waypoint=*state;double delta=0;
      for(std::size_t i=0;i<trajectory.joint_names.size();++i){waypoint.setVariablePosition(trajectory.joint_names[i],point.positions.at(i));delta=std::max(delta,std::abs(point.positions[i]-previous.getVariablePosition(trajectory.joint_names[i])));}
      const int steps=std::max(1,int(std::ceil(delta/0.005)));
      for(int i=1;i<=steps;++i){auto sample=previous;previous.interpolate(waypoint,double(i)/steps,sample);
        if(!valid(scene,sample,acm,log,"Pre-grasp trajectory",false)){valid(scene,sample,acm,log,"Invalid trajectory",true);throw std::runtime_error("APPROACH_COLLISION pre-grasp path");}++samples;}
      previous=waypoint;
    }
    RCLCPP_INFO(log,"Pre-grasp trajectory validated samples=%zu no touch exemptions",samples);
    auto approach_start=previous;
    if(execute){
      auto fresh=*state;fresh_state(fresh,reader,node);
      for(const auto& name:fresh.getVariableNames())require(std::abs(fresh.getVariablePosition(name)-state->getVariablePosition(name))<0.005,"Start state changed; refusing execution");
      require(group.execute(plan)==moveit::core::MoveItErrorCode::SUCCESS,"EXECUTION_FAILED pre-grasp");
      double pe=1,oe=1;
      for(int i=0;i<500;++i){fresh_state(approach_start,reader,node);
        const auto observed=approach_start.getGlobalLinkTransform("panda1_tcp");const auto desired=transform(candidate.pre_grasp_pose);
        pe=(observed.translation()-desired.translation()).norm();oe=Eigen::Quaterniond(observed.linear()).angularDistance(Eigen::Quaterniond(desired.linear()));
        if(pe<=0.003&&oe<=0.01)break;
      }
      RCLCPP_INFO(log,"Pre-grasp execution=SUCCESS TCP position_error=%.12f m orientation_error=%.12f rad",pe,oe);
      require(pe<=0.01&&oe<=0.03,"EXECUTION_FAILED final TCP error");
      require(valid(scene,approach_start,acm,log,"Final pre-grasp",true),"Invalid executed final state");
    }else RCLCPP_INFO(log,"Pre-grasp execution skipped (plan only)");
    if(check_grasp){
      const auto grasp_snapshot=get_scene();
      require(grasp_snapshot.allowed_collision_matrix==snapshot.allowed_collision_matrix,"Global ACM changed before grasp evaluation");
      scene.setPlanningSceneMsg(grasp_snapshot);
      const auto grasp_cube=cube_pose(scene);
      require((transform(grasp_cube).translation()-transform(cube).translation()).norm()<=0.002 &&
        Eigen::Quaterniond(transform(grasp_cube).linear()).angularDistance(Eigen::Quaterniond(transform(cube).linear()))<=0.01,
        "SCENE_SYNC_FAILED cube moved before grasp evaluation");
      const auto touch=hm::graspEvaluationACM(acm,candidate);
      RCLCPP_INFO(log,"Temporary local touch policy: cube <-> panda1_left_finger, panda1_right_finger ONLY");
      auto grasp=approach_start;require(solve(grasp,candidate.grasp_pose,touch,false),"IK_FAILED grasp");
      print_joints(log,"Grasp IK",grasp);valid(scene,grasp,acm,log,"Grasp NORMAL",true);
      require(valid(scene,grasp,touch,log,"Grasp ALLOWED_TOUCH",true),"Grasp invalid even with minimal touch");
      auto sample=approach_start;
      for(int i=0;i<=20;++i){auto target=candidate.pre_grasp_pose;
        target.position.x+=(candidate.grasp_pose.position.x-target.position.x)*i/20.0;
        target.position.y+=(candidate.grasp_pose.position.y-target.position.y)*i/20.0;
        target.position.z+=(candidate.grasp_pose.position.z-target.position.z)*i/20.0;
        const auto& policy=i>=18?touch:acm;const auto before=sample;
        require(solve(sample,target,policy,true),"APPROACH_COLLISION/IK_FAILED sampled approach");
        for(int sub=1;sub<=10;++sub){auto interpolated=before;before.interpolate(sample,sub/10.0,interpolated);
          require(valid(scene,interpolated,policy,log,"Approach interpolation",false),"APPROACH_COLLISION joint interpolation");}
        require(valid(scene,sample,policy,log,"Approach sample "+std::to_string(i),true),"Approach invalid");
        if(i>=18)valid(scene,sample,acm,log,"Approach final region NORMAL "+std::to_string(i),true);
      }
      RCLCPP_INFO(log,"Approach validation PASS: 21 Cartesian IK samples + 210 joint samples; NOT EXECUTED");
    }
    fresh_state(*state,reader,node);auto final_snapshot=get_scene();
    require(final_snapshot.allowed_collision_matrix==snapshot.allowed_collision_matrix,"Global ACM changed");
    require(final_snapshot.robot_state.attached_collision_objects.empty(),"Unexpected attachment");
    scene.setPlanningSceneMsg(final_snapshot);cube_pose(scene);
    require(scene.getWorld()->hasObject("cube"),"Cube no longer world object");
    require(drift->load()<=0.002&&gripper_drift->load()<=0.001,"Stationary robot/gripper drift exceeded tolerance");
    RCLCPP_INFO(log,"Panda2 max drift=%.12f Panda1 finger max drift=%.12f; cube WORLD; no gripper/attach/approach commands",drift->load(),gripper_drift->load());
    RCLCPP_INFO(log,"VALIDATION PASS execute_pregrasp=%s validate_grasp=%s",execute?"true":"false",check_grasp?"true":"false");result=0;
    if(keep_alive){RCLCPP_INFO(log,"Keeping debug markers alive; Ctrl+C exits without robot commands");while(rclcpp::ok())std::this_thread::sleep_for(100ms);}
  }catch(const std::exception& e){RCLCPP_ERROR(log,"VALIDATION FAIL: %s",e.what());}
  executor.cancel();spinner.join();if(rclcpp::ok())rclcpp::shutdown();return result;
}
