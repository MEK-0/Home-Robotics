#include "home_robotics_manipulation/validation_utils.hpp"
#include "home_robotics_manipulation/gripper_interface.hpp"
#include "home_robotics_manipulation/cube_demo_policy.hpp"
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/string.hpp>
#include "home_robotics_manipulation/placement_target.hpp"
#include <fstream>
#include <optional>
#ifndef PHASE4_PICK_PLACE
#define PHASE4_PICK_PLACE 0
#endif

using Plan=moveit::planning_interface::MoveGroupInterface::Plan;
using S=hm::ManipulationState;
namespace {
template<class Service>
auto call(const rclcpp::Node::SharedPtr& node,const std::string& name,
          typename Service::Request::SharedPtr request) {
  auto client=node->create_client<Service>(name);
  require(client->wait_for_service(5s),"Missing service "+name);
  auto future=client->async_send_request(request);
  require(future.wait_for(10s)==std::future_status::ready,"Service timeout "+name);
  return future.get();
}
geometry_msgs::msg::Pose toPose(const Eigen::Isometry3d& t) {
  geometry_msgs::msg::Pose p; p.position.x=t.translation().x();p.position.y=t.translation().y();p.position.z=t.translation().z();
  Eigen::Quaterniond q(t.linear());p.orientation.x=q.x();p.orientation.y=q.y();p.orientation.z=q.z();p.orientation.w=q.w();return p;
}
}
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  auto node=rclcpp::Node::make_shared("cube_pick_lift_return_demo",rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto reader=rclcpp::Node::make_shared("cube_demo_reader"); auto log=node->get_logger();
  rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(node);
  std::thread spinner([&](){executor.spin();});int result=1; S stage=S::IDLE;std::string stage_name="IDLE";
  auto transition=[&](S next,const char* name){require(hm::transition_allowed(stage,next),"Invalid manipulation transition");stage=next;stage_name=name;RCLCPP_INFO(log,"STATE -> %s",name);};
  bool execute=false,stabilized=false,attached=false,paused=false;
  std::string target_id,result_file,object;
  YAML::Node report;report["success"]=false;report["grasp_verified"]=false;
  report["lift_success"]=false;report["transport_success"]=false;report["placement_success"]=false;
  double planning_seconds=0;auto started=std::chrono::steady_clock::now();
  node->get_parameter_or<std::string>("result_file",result_file,"");
  node->get_parameter_or<std::string>("target",target_id,PHASE4_PICK_PLACE?"surface_left_2":"");
  report["target"]=target_id;
  try {
    std::string robot;double height;
    node->get_parameter_or<std::string>("robot",robot,"panda1");node->get_parameter_or<std::string>("object",object,"cube");
    node->get_parameter_or("execute",execute,false);node->get_parameter_or("lift_height",height,0.10);
    report["object"]=object;
    require(!PHASE4_PICK_PLACE || !target_id.empty(),"PLACE_FAILED named target required");
    require(robot=="panda1"&&(object=="cube"||object=="purple_ball"),"Only Panda1 cube/purple_ball supported");require(hm::validLiftHeight(height),"lift_height must be finite and 0.08..0.15 m");
    moveit::planning_interface::MoveGroupInterface group(node,"panda1_manipulator");
    group.setEndEffectorLink("panda1_tcp");group.setPoseReferenceFrame("world");group.setPlanningPipelineId("ompl");group.setPlannerId("RRTConnectkConfigDefault");
    group.setPlanningTime(15);group.setMaxVelocityScalingFactor(0.05);group.setMaxAccelerationScalingFactor(0.05);
    hm::GripperInterface gripper(node);
    moveit::core::RobotState state(group.getRobotModel());fresh_state(state,reader,node);const auto baseline=state;
    auto panda2_drift=std::make_shared<std::atomic<double>>(0);
    auto monitor=node->create_subscription<sensor_msgs::msg::JointState>("/joint_states",10,[baseline,panda2_drift](sensor_msgs::msg::JointState::ConstSharedPtr msg){
      for(size_t i=0;i<msg->name.size()&&i<msg->position.size();++i)if(msg->name[i].rfind("panda2_",0)==0){
        double d=std::abs(msg->position[i]-baseline.getVariablePosition(msg->name[i])),old=panda2_drift->load();
        while(d>old&&!panda2_drift->compare_exchange_weak(old,d)){}
      }
    });
    auto get_scene=[&](){auto q=std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();q->components.components=1023;
      return call<moveit_msgs::srv::GetPlanningScene>(node,"/get_planning_scene",q)->scene;};
    auto apply=[&](const moveit_msgs::msg::PlanningScene& scene){auto q=std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();q->scene=scene;
      require(call<moveit_msgs::srv::ApplyPlanningScene>(node,"/apply_planning_scene",q)->success,"SCENE_SYNC_FAILED apply");};
    auto bool_service=[&](const std::string& name,bool value){auto q=std::make_shared<std_srvs::srv::SetBool::Request>();q->data=value;
      auto r=call<std_srvs::srv::SetBool>(node,name,q);require(r->success,name+": "+r->message);return r->message;};
    auto trigger=[&](const std::string& name){auto r=call<std_srvs::srv::Trigger>(node,name,std::make_shared<std_srvs::srv::Trigger::Request>());
      require(r->success,name+": "+r->message);RCLCPP_INFO(log,"%s %s",name.c_str(),r->message.c_str());return YAML::Load(r->message);};
    auto pause=[&](bool value){
      const auto deadline=std::chrono::steady_clock::now()+10s;
      while(true){auto q=std::make_shared<std_srvs::srv::SetBool::Request>();q->data=value;
        auto r=call<std_srvs::srv::SetBool>(node,"/dynamic_object_scene_sync/pause",q);
        paused=value;if(r->success)break;require(std::chrono::steady_clock::now()<deadline,"SCENE_SYNC_FAILED drain timeout");std::this_thread::sleep_for(50ms);}
    };
    auto contact=[&](){std_msgs::msg::String message;
      require(rclcpp::wait_for_message(message,reader,"/mujoco/objects/"+object+"/contact",2s),"NO_CONTACT missing contact interface");
      auto c=YAML::Load(message.data);double age=node->now().seconds()-c["timestamp"].as<double>();
      require(age>=-0.1&&age<=0.5,"OBJECT_DROPPED stale physics state");require(c["object_id"].as<std::string>()==object,"Wrong contact object");return c;};
    auto pose=[&](){geometry_msgs::msg::PoseStamped p;
      require(rclcpp::wait_for_message(p,reader,"/mujoco/objects/"+object+"/pose",2s),"OBJECT_NOT_FOUND cube");
      double age=(node->now()-rclcpp::Time(p.header.stamp)).seconds();require(age>=-0.1&&age<=0.5&&p.header.frame_id=="world","Stale/wrong cube pose");return p;};
    auto controller_gate=[&](){auto r=call<controller_manager_msgs::srv::ListControllers>(node,"/controller_manager/list_controllers",std::make_shared<controller_manager_msgs::srv::ListControllers::Request>());
      for(const auto& name:{"panda1_trajectory_controller","panda1_gripper_controller"}){
        bool ok=false;for(const auto& c:r->controller)if(c.name==name&&c.state=="active")ok=true;require(ok,std::string("Inactive controller ")+name);}
    };
    const auto config=ament_index_cpp::get_package_share_directory("home_robotics_control")+"/config/";
    hm::ObjectRegistry registry(config+"objects.yaml");auto initial_pose=pose();print_pose(log,"Initial object",initial_pose.pose);
    planning_scene::PlanningScene scene(group.getRobotModel());auto initial_scene=get_scene();scene.setPlanningSceneMsg(initial_scene);
    const auto original_acm=scene.getAllowedCollisionMatrix();
    const auto support_id=YAML::LoadFile(config+"objects.yaml")["objects"][object]["initial"]["support_surface"].as<std::string>();
    std::optional<hm::PlacementTarget> placement;
    auto desired_cube=initial_pose.pose;
    if(!target_id.empty()) {
      require(target_id!=support_id,"PLACE_FAILED target must differ from source surface");
      placement=hm::PlacementTarget::lookup(config+"scene.yaml",target_id);
      desired_cube=placement->objectPose(registry.at(object),initial_pose.pose.orientation);
      print_pose(log,"Target cube",desired_cube);
    }
    auto supported=[&](const YAML::Node& c,bool proximity){
      if(!placement)return c["support_contact"].as<bool>();
      const auto list=c[proximity?"near_support_surfaces":"support_surfaces"];
      for(const auto& id:list)if(id.as<std::string>()==target_id)return true;
      return false;
    };
    auto checked=[&](planning_scene::PlanningScene& sc,moveit::core::RobotState& st,
                     const collision_detection::AllowedCollisionMatrix& matrix,const rclcpp::Logger& logger,
                     const std::string& label,bool verbose){
      st.update();collision_detection::CollisionRequest request;collision_detection::CollisionResult response;
      request.contacts=true;request.max_contacts=1000;request.max_contacts_per_pair=100;
      sc.checkCollision(request,response,st,matrix);
      bool ok=st.satisfiesBounds(1e-6)&&separation(st)>=0.7;
      if(response.collision){
        // Physics rests the cube ~0.108 mm into the table. Check, rather than disable,
        // the exact support contact at the original placement region only.
        const auto* body=st.getAttachedBody(object);
        bool support_region=false,destination_region=false;
        if(body){const auto xyz=body->getGlobalPose().translation();
          support_region=(xyz.head<2>()-transform(initial_pose.pose).translation().head<2>()).norm()<0.03 &&
            std::abs(xyz.z()-initial_pose.pose.position.z)<0.001;
          if(placement)destination_region=placement->contains(registry.at(object),toPose(body->getGlobalPose()));}
        if(response.contacts.empty())ok=false;
        for(const auto& [pair,contacts]:response.contacts){
          bool support=(support_region&&((pair.first==object&&pair.second==support_id)||(pair.second==object&&pair.first==support_id))) ||
            (destination_region&&((pair.first==object&&pair.second==target_id)||(pair.second==object&&pair.first==target_id)));
          for(const auto& c:contacts)if(!support||c.depth>0.0005)ok=false;
          if(verbose||!ok)RCLCPP_INFO(logger,"%s contact %s <-> %s depth=%.9f accepted_support=%d",label.c_str(),pair.first.c_str(),pair.second.c_str(),contacts.empty()?0:contacts.front().depth,support&&ok);
        }
      }
      return ok;
    };
    hm::DemoReadiness readiness;
    auto gate=[&](){
      // Fresh stamps alone do not prove that simulation time is advancing.
      auto clock_before=node->now();std::this_thread::sleep_for(100ms);
      require(node->now()>clock_before,"INVALID_OBJECT_STATE stopped /clock");
      controller_gate();fresh_state(state,reader,node);auto msg=get_scene();scene.setPlanningSceneMsg(msg);
      require(msg.robot_state.attached_collision_objects.empty(),"Initial objects must all be WORLD");
      std::set<std::string> ids={"arena_front","arena_back","arena_left","arena_right"};auto cfg=YAML::LoadFile(config+"scene.yaml")["scene"];
      ids.insert(cfg["floor"]["id"].as<std::string>());
      for(const auto& x:cfg["surfaces"])ids.insert(x.first.as<std::string>());
      for(const auto& x:cfg["shared_rail"]["supports"]["names"])ids.insert(x.as<std::string>());
      for(const auto& [id,metadata]:registry.objects())ids.insert(id);
      for(const auto& id:ids)require(scene.getWorld()->hasObject(id),"SCENE_SYNC_FAILED missing "+id);
      for(const auto& [id,metadata]:registry.objects()){
        geometry_msgs::msg::PoseStamped object_pose;
        require(rclcpp::wait_for_message(object_pose,reader,"/mujoco/objects/"+id+"/pose",2s),"SCENE_SYNC_FAILED missing pose "+id);
        require(std::abs((node->now()-rclcpp::Time(object_pose.header.stamp)).seconds())<=0.5,"SCENE_SYNC_FAILED stale pose "+id);
        auto actual=transform(hm::world_object(metadata,object_pose).pose);
        auto represented=scene.getWorld()->getObject(id)->pose_;
        require((actual.translation()-represented.translation()).norm()<0.002,"SCENE_SYNC_FAILED dynamic pose "+id);
      }
      auto p=pose();auto cube=scene.getWorld()->getObject(object);
      require((cube->pose_.translation()-transform(p.pose).translation()).norm()<=0.002,"SCENE_SYNC_FAILED cube translation");
      require(Eigen::Quaterniond(cube->pose_.linear()).angularDistance(Eigen::Quaterniond(transform(p.pose).linear()))<=0.01,"SCENE_SYNC_FAILED cube rotation");
      for(const auto& link:group.getRobotModel()->getLinkModelNames()){
        collision_detection::AllowedCollision::Type type;
        require(!scene.getAllowedCollisionMatrix().getAllowedCollision(link,object,type)||type==collision_detection::AllowedCollision::NEVER,"Unexpected global cube allowance");}
      require(checked(scene,state,original_acm,log,"Initial full state",true),"Initial collision/bounds/rail gate failed");
      require(!contact()["stabilization_active"].as<bool>(),"Stale stabilization active");
      readiness={true,true,true,true,true,true,true,true,true,separation(state)};
      require(readiness.ready(),"Incomplete readiness gate");
      RCLCPP_INFO(log,"SAFETY GATE PASS controllers, fresh states, static scene, five objects, unique WORLD cube, collision and rail separation");
    };gate();transition(S::OBJECT_SELECTED,"OBJECT_SELECTED");
    hm::GraspParameters parameters;parameters.finger_open_width=object=="cube"?registry.at(object).geometry.primitives[0].dimensions[0]+0.014:hm::pandaGripperGeometry(group.getRobotModel()).maximum_open_width;
    auto candidate=hm::generateTopDownGrasp(registry.at(object),initial_pose.pose,hm::pandaGripperGeometry(group.getRobotModel()),parameters);
    auto touch=hm::graspEvaluationACM(original_acm,candidate);
    const auto* jmg=state.getJointModelGroup("panda1_manipulator");
    auto validate_plan=[&](const Plan& plan,const moveit::core::RobotState& start,const collision_detection::AllowedCollisionMatrix& acm){
      const auto& tr=plan.trajectory.joint_trajectory;require(hm::pregraspJointNamesAllowed(tr.joint_names)&&!tr.points.empty(),"Unsafe/empty trajectory joints");
      auto previous=start;
      for(const auto& point:tr.points){auto next=previous;double delta=0;
        for(size_t i=0;i<tr.joint_names.size();++i){next.setVariablePosition(tr.joint_names[i],point.positions.at(i));delta=std::max(delta,std::abs(point.positions[i]-previous.getVariablePosition(tr.joint_names[i])));}
        int n=std::max(1,int(std::ceil(delta/0.005)));
        for(int i=1;i<=n;++i){auto sample=previous;previous.interpolate(next,double(i)/n,sample);if(!checked(scene,sample,acm,log,"Path",false)){checked(scene,sample,acm,log,"Rejected path",true);throw std::runtime_error("APPROACH_COLLISION sampled path");}}previous=next;
      }previous.update();return previous;
    };
    auto cartesian=[&](const moveit::core::RobotState& start,const geometry_msgs::msg::Pose& target,const collision_detection::AllowedCollisionMatrix& acm){
      auto planning_started=std::chrono::steady_clock::now();
      // Sequential Cartesian IK with bounded joint changes, then time parameterization.
      auto sample=start;auto begin=start.getGlobalLinkTransform("panda1_tcp");auto end=transform(target);
      int n=std::max(2,int(std::ceil((end.translation()-begin.translation()).norm()/0.002)));
      robot_trajectory::RobotTrajectory tr(group.getRobotModel(),"panda1_manipulator");tr.addSuffixWayPoint(start,0);
      for(int i=1;i<=n;++i){Eigen::Isometry3d desired=Eigen::Isometry3d::Identity();double f=double(i)/n;
        desired.translation()=(1-f)*begin.translation()+f*end.translation();desired.linear()=Eigen::Quaterniond(begin.linear()).slerp(f,Eigen::Quaterniond(end.linear())).toRotationMatrix();
        auto previous=sample;auto cb=[&](moveit::core::RobotState* s,const moveit::core::JointModelGroup* g,const double* values){s->setJointGroupPositions(g,values);s->update();
          for(const auto& name:g->getVariableNames())if(std::abs(s->getVariablePosition(name)-previous.getVariablePosition(name))>0.15)return false;
          return checked(scene,*s,acm,log,"Cartesian IK",false);};
        bool found=false;for(int attempt=0;attempt<8&&!found;++attempt){sample=previous;found=sample.setFromIK(jmg,desired,"panda1_tcp",0.2,cb);}
        require(found,"IK_FAILED/APPROACH_COLLISION Cartesian sample "+std::to_string(i));tr.addSuffixWayPoint(sample,0.1);
      }
      trajectory_processing::TimeOptimalTrajectoryGeneration timing(0.000001, 0.02, 0.0001);require(timing.computeTimeStamps(tr,0.04,0.04),"PLANNING_FAILED trajectory retiming");
      Plan p;moveit::core::robotStateToRobotStateMsg(start,p.start_state);tr.getRobotTrajectoryMsg(p.trajectory);validate_plan(p,start,acm);
      planning_seconds+=std::chrono::duration<double>(std::chrono::steady_clock::now()-planning_started).count();
      RCLCPP_INFO(log,"Cartesian plan SUCCESS samples=%d points=%zu",n,p.trajectory.joint_trajectory.points.size());return p;
    };
    auto run=[&](const Plan& p,const collision_detection::AllowedCollisionMatrix& acm,const char* label){
      fresh_state(state,reader,node);auto snapshot=get_scene();scene.setPlanningSceneMsg(snapshot);
      auto expected=state;moveit::core::robotStateMsgToRobotState(p.start_state,expected);
      for(const auto& name:state.getVariableNames())require(std::abs(state.getVariablePosition(name)-expected.getVariablePosition(name))<=0.01,"EXECUTION_FAILED changed start state");
      auto target=validate_plan(p,state,acm).getGlobalLinkTransform("panda1_tcp");
      require(panda2_drift->load()<=0.002,"Panda2 drift");
      // Poll physics while the action runs; abort immediately on a lost attachment.
      auto execution=std::async(std::launch::async,[&](){return group.execute(p);});
      struct StopOnError {
        moveit::planning_interface::MoveGroupInterface& group;
        int exceptions=std::uncaught_exceptions();
        ~StopOnError(){if(std::uncaught_exceptions()>exceptions)group.stop();}
      } stop_on_error{group};
      auto deadline=std::chrono::steady_clock::now()+60s;double pe=1,oe=1;
      while(std::chrono::steady_clock::now()<deadline){
        fresh_state(state,reader,node);
        require(panda2_drift->load()<=0.002,"Panda2 drift during motion");
        if(stabilized){auto c=contact();if(!c["stabilization_active"].as<bool>()||c["relative_drift"].as<double>()>0.01){group.stop();throw std::runtime_error("OBJECT_DROPPED grasp relation");}require(c["unexpected_contacts"].size()==0,"APPROACH_COLLISION unexpected physical contact; retaining grasp");}
        auto actual=state.getGlobalLinkTransform("panda1_tcp");pe=(actual.translation()-target.translation()).norm();oe=Eigen::Quaterniond(actual.linear()).angularDistance(Eigen::Quaterniond(target.linear()));
        if(pe<0.003&&oe<0.015&&execution.wait_for(0s)==std::future_status::ready)break;
        if(execution.wait_for(0s)==std::future_status::ready && pe>0.02){group.stop();throw std::runtime_error("EXECUTION_FAILED stopped short");}
      }
      if(pe>=0.003||oe>=0.015){group.stop();throw std::runtime_error("EXECUTION_FAILED TCP tracking timeout");}
      // Ensure controller has finished before issuing the next trajectory.
      require(execution.get()==moveit::core::MoveItErrorCode::SUCCESS,"EXECUTION_FAILED controller result");
      if(stabilized)report["cube_gripper_max_drift"]=contact()["relative_drift"].as<double>();
      RCLCPP_INFO(log,"%s execution SUCCESS TCP_error=%.9f orientation_error=%.9f",label,pe,oe);
    };
    // Pre-grasp is planned from the measured full state. Opening is inside the execution gate.
    group.setStartState(state);group.setPoseTarget(candidate.pre_grasp_pose,"panda1_tcp");Plan pre;
    require(group.plan(pre)==moveit::core::MoveItErrorCode::SUCCESS,"PLANNING_FAILED pre-grasp");auto pre_end=validate_plan(pre,state,original_acm);
    transition(S::PREGRASP_PLANNED,"PREGRASP_PLANNED");
    RCLCPP_INFO(log,"Pre-grasp plan SUCCESS time=%.9f points=%zu",pre.planning_time,pre.trajectory.joint_trajectory.points.size());
    planning_seconds+=pre.planning_time;
    // Validate the full hypothetical attached route before ANY physical command.
    auto preview=[&](){
      auto approach=cartesian(pre_end,candidate.grasp_pose,touch);auto grasp=validate_plan(approach,pre_end,touch);
      // Hypothetical attached geometry is confined to this local scene; no ROS mutations.
      moveit_msgs::msg::AttachedCollisionObject a;a.link_name="panda1_hand";a.touch_links=candidate.allowed_touch_links;
      a.object=hm::world_object(registry.at(object),initial_pose);a.object.header.frame_id="panda1_hand";
      a.object.pose=toPose(grasp.getGlobalLinkTransform("panda1_hand").inverse()*transform(initial_pose.pose));
      scene.getCurrentStateNonConst()=grasp;scene.processAttachedCollisionObjectMsg(a);grasp=scene.getCurrentState();
      auto target=candidate.grasp_pose;target.position.z+=height;
      auto lift=cartesian(grasp,target,original_acm);auto lifted=validate_plan(lift,grasp,original_acm);
      auto place_tcp=toPose(transform(desired_cube)*transform(initial_pose.pose).inverse()*transform(candidate.grasp_pose));
      if(placement){
        auto above=placement->prePlace(place_tcp);
        auto transport=cartesian(lifted,above,original_acm);
        auto at_destination=validate_plan(transport,lifted,original_acm);
        cartesian(at_destination,place_tcp,original_acm);
        print_pose(log,"Pre-place TCP",above);print_pose(log,"Place TCP",place_tcp);
      }else cartesian(lifted,candidate.grasp_pose,original_acm);
      scene.setPlanningSceneMsg(get_scene());
    };
    preview();
    if(!execute){
      report["planning_only_pass"]=true;
      // A completed preflight is a successful task outcome. No final physical
      // pose is claimed because the robot and object were deliberately not moved.
      report["success"]=true;
      report["planning_only"]=true;
      report["failure_reason"]="NONE";
      report["failure_detail"]="Planning-only preflight passed; no physical commands issued";
      RCLCPP_INFO(log,"PLANNING_ONLY PASS: no physical commands, services mutating physics, or remote scene changes; physical contact not verified");result=0;
    }else{
      gate();require((transform(pose().pose).translation()-transform(initial_pose.pose).translation()).norm()<0.002,"Cube moved since selection");
      require(hm::physicalCommandsAllowed(execute,readiness),"Execution disabled or safety gate incomplete");
      gripper.open(candidate.expected_gripper_width);fresh_state(state,reader,node);
      require(std::abs(state.getVariablePosition("panda1_finger_joint1")+state.getVariablePosition("panda1_finger_joint2")-candidate.expected_gripper_width)<0.004,"GRIPPER_FAILED opening");
      // Replan after opening so the full start state is authoritative.
      group.setStartState(state);group.setPoseTarget(candidate.pre_grasp_pose,"panda1_tcp");require(group.plan(pre)==moveit::core::MoveItErrorCode::SUCCESS,"PLANNING_FAILED pre-grasp after opening");
      run(pre,original_acm,"Pre-grasp");transition(S::APPROACHING,"APPROACHING");
      auto approach=cartesian(state,candidate.grasp_pose,touch);run(approach,touch,"Approach");
      trigger("/mujoco/"+object+"/begin_grasp");transition(S::GRIPPER_CLOSING,"GRIPPER_CLOSING");gripper.close();
      RCLCPP_INFO(log,"Gripper close action SUCCESS (physical verification still required)");
      transition(S::GRASP_VERIFYING,"GRASP_VERIFYING");
      try{trigger("/mujoco/"+object+"/verify_grasp");}catch(...){
        // Remain at the supported original location; open and validate a retreat.
        auto c=contact();if(c["support_contact"].as<bool>()){
          gripper.open(candidate.expected_gripper_width);fresh_state(state,reader,node);scene.setPlanningSceneMsg(get_scene());
          auto recovery=cartesian(state,candidate.pre_grasp_pose,touch);run(recovery,touch,"Failure retreat");}
        throw;
      }
      transition(S::GRASP_VERIFIED,"GRASP_VERIFIED");report["grasp_verified"]=true;
      RCLCPP_INFO(log,"Stabilize %s",bool_service("/mujoco/"+object+"/stabilize",true).c_str());stabilized=true;
      // Observe a short simulation-time settling interval and enforce activation displacement.
      auto c=contact();double t=c["timestamp"].as<double>();auto settle_deadline=std::chrono::steady_clock::now()+5s;do{require(std::chrono::steady_clock::now()<settle_deadline,"INVALID_OBJECT_STATE stopped physics clock");c=contact();require(c["activation_jump"].as<double>()<=0.005,"GRASP_UNSTABLE activation jump");}while(c["timestamp"].as<double>()-t<0.12);
      RCLCPP_INFO(log,"Activation pose jump=%.12f",c["activation_jump"].as<double>());
      pause(true);fresh_state(state,reader,node);auto current_cube=pose();
      moveit_msgs::msg::PlanningScene diff;diff.is_diff=true;diff.robot_state.is_diff=true;
      moveit_msgs::msg::CollisionObject remove;remove.id=object;remove.operation=remove.REMOVE;diff.world.collision_objects.push_back(remove);
      moveit_msgs::msg::AttachedCollisionObject a;a.link_name="panda1_hand";a.touch_links=candidate.allowed_touch_links;
      a.object=hm::world_object(registry.at(object),current_cube);a.object.header.frame_id="panda1_hand";
      a.object.pose=toPose(state.getGlobalLinkTransform("panda1_hand").inverse()*transform(current_cube.pose));diff=hm::attachCube(a,stage==S::GRASP_VERIFIED,stabilized);apply(diff);
      auto attached_scene=get_scene();int count=0;for(const auto& x:attached_scene.robot_state.attached_collision_objects)if(x.object.id==object)++count;
      for(const auto& x:attached_scene.world.collision_objects)require(x.id!=object,"Duplicate WORLD cube");require(count==1,"Attachment not unique");attached=true;pause(false);RCLCPP_INFO(log,"WORLD -> ATTACHED verified: world_cube=0 attached_cube=1");
      transition(S::OBJECT_ATTACHED,"OBJECT_ATTACHED");scene.setPlanningSceneMsg(get_scene());
      fresh_state(state,reader,node);state=scene.getCurrentState();fresh_state(state,reader,node);
      const auto grasp_tcp=state.getGlobalLinkTransform("panda1_tcp");const auto grasp_cube=pose();
      auto lift_pose=toPose(grasp_tcp);lift_pose.position.z+=height;transition(S::LIFTING,"LIFTING");
      auto lift=cartesian(state,lift_pose,original_acm);run(lift,original_acm,"Lift");
      auto lifted_cube=pose();c=contact();double tcp_lift=state.getGlobalLinkTransform("panda1_tcp").translation().z()-grasp_tcp.translation().z();
      double cube_lift=lifted_cube.pose.position.z-grasp_cube.pose.position.z;
      RCLCPP_INFO(log,"LIFT requested=%.9f TCP=%.9f cube=%.9f relative_drift=%.12f support=%s",height,tcp_lift,cube_lift,c["relative_drift"].as<double>(),c["support_contact"].as<bool>()?"true":"false");
      require(cube_lift>=0.08&&std::abs(tcp_lift-height)<=0.01&&!c["support_contact"].as<bool>()&&c["relative_drift"].as<double>()<=0.01,"OBJECT_DROPPED lift verification");
      report["lift_success"]=true;report["cube_lift"]=cube_lift;
      transition(S::TRANSPORTING,"TRANSPORTING");t=c["timestamp"].as<double>();
      auto hold_deadline=std::chrono::steady_clock::now()+5s;
      do{require(std::chrono::steady_clock::now()<hold_deadline,"INVALID_OBJECT_STATE stopped physics clock");c=contact();require(c["stabilization_active"].as<bool>()&&c["relative_drift"].as<double>()<=0.01,"OBJECT_DROPPED hold");}while(c["timestamp"].as<double>()-t<1.5);
      auto place_tcp=toPose(grasp_tcp);
      if(placement){
        place_tcp=toPose(transform(desired_cube)*transform(grasp_cube.pose).inverse()*grasp_tcp);
        auto above=placement->prePlace(place_tcp);
        scene.setPlanningSceneMsg(get_scene());fresh_state(state,reader,node);
        auto transport=cartesian(state,above,original_acm);
        transition(S::PREPLACE_PLANNED,"PREPLACE_PLANNED");
        run(transport,original_acm,"Transport");
        report["transport_success"]=true;
      }else transition(S::PREPLACE_PLANNED,"PREPLACE_PLANNED");
      transition(S::PLACING,"PLACING");
      scene.setPlanningSceneMsg(get_scene());fresh_state(state,reader,node);
      auto descent=cartesian(state,place_tcp,original_acm);run(descent,original_acm,"Place descent");
      c=contact();require(supported(c,false)||supported(c,true),"PLACE_FAILED no destination support/proximity; holding grasp");
      require(!placement || placement->contains(registry.at(object),pose().pose),"PLACE_FAILED outside support region");
      report["support_contact_before_release"]=supported(c,false);
      report["support_proximity_before_release"]=supported(c,true);
      RCLCPP_INFO(log,"Support verified contact=%d proximity=%d",supported(c,false),supported(c,true));
      transition(S::RELEASING,"RELEASING");bool_service("/mujoco/"+object+"/stabilize",false);stabilized=false;RCLCPP_INFO(log,"Stabilization removed after support verification");
      pause(true);diff=moveit_msgs::msg::PlanningScene();diff.is_diff=true;diff.robot_state.is_diff=true;
      a=moveit_msgs::msg::AttachedCollisionObject();a.link_name="panda1_hand";a.object.id=object;a.object.operation=a.object.REMOVE;
      diff=hm::releaseCube(hm::world_object(registry.at(object),pose()),supported(c,false)||supported(c,true),stabilized);apply(diff);
      auto detached=get_scene();count=0;for(const auto& x:detached.world.collision_objects)if(x.id==object)++count;
      for(const auto& x:detached.robot_state.attached_collision_objects)require(x.object.id!=object,"Stale attached cube");require(count==1,"WORLD cube not unique");attached=false;pause(false);RCLCPP_INFO(log,"ATTACHED -> WORLD verified: world_cube=1 attached_cube=0");
      gripper.open(candidate.expected_gripper_width);fresh_state(state,reader,node);scene.setPlanningSceneMsg(get_scene());state=scene.getCurrentState();fresh_state(state,reader,node);
      report["gripper_open"]=true;report["stabilization_released"]=!stabilized;
      RCLCPP_INFO(log,"Gripper open action SUCCESS");
      auto retreat_pose=placement?placement->prePlace(place_tcp):candidate.pre_grasp_pose;
      auto retreat=cartesian(state,retreat_pose,touch);run(retreat,touch,"Retreat");report["retreat_success"]=true;
      transition(S::PLACE_VERIFYING,"PLACE_VERIFYING");auto final_pose=pose();print_pose(log,"Final object",final_pose.pose);
      double pe=(transform(final_pose.pose).translation()-transform(desired_cube).translation()).norm();
      double oe=Eigen::Quaterniond(transform(final_pose.pose).linear()).angularDistance(Eigen::Quaterniond(transform(desired_cube).linear()));
      c=contact();require(pe<=0.03&&supported(c,false)&&!c["stabilization_active"].as<bool>(),"PLACE_FAILED final pose/support");
      require(!placement||placement->contains(registry.at(object),final_pose.pose),"PLACE_FAILED final region");
      fresh_state(state,reader,node);
      require(std::abs(state.getVariablePosition("panda1_finger_joint1")+state.getVariablePosition("panda1_finger_joint2")-candidate.expected_gripper_width)<0.004,"GRIPPER_FAILED final open width");
      auto final_scene=get_scene();int world_count=0,attached_count=0;
      for(const auto& x:final_scene.world.collision_objects)if(x.id==object)++world_count;
      for(const auto& x:final_scene.robot_state.attached_collision_objects)if(x.object.id==object)++attached_count;
      require(world_count==1&&attached_count==0,"SCENE_SYNC_FAILED final ownership");
      require(panda2_drift->load()<=0.002,"Panda2 drift");
      report["placement_success"]=true;report["success"]=true;
      report["final_position_error"]=pe;report["final_orientation_error"]=oe;
      report["panda2_max_drift"]=panda2_drift->load();
      report["world_count"]=world_count;report["attached_count"]=attached_count;
      for(auto item:{std::make_pair("target_pose",desired_cube),std::make_pair("final_pose",final_pose.pose)}){
        auto p=item.second;report[item.first]=std::vector<double>{p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w};}
      RCLCPP_INFO(log,"FINAL position_error=%.12f orientation_error=%.12f Panda2_max_drift=%.12f",pe,oe,panda2_drift->load());
      transition(S::PLACE_VERIFIED,"PLACE_VERIFIED");transition(S::DONE,"DONE");RCLCPP_INFO(log,"%s %s PASS",object.c_str(),placement?"PICK PLACE":"PICK LIFT RETURN");result=0;
    }
  }catch(const std::exception& e){
    report["failure_stage"]=stage_name;report["failure_reason"]=hm::failure_code(e.what(),stage);report["failure_detail"]=std::string(e.what());
    // A place failure retains the healthy grasp. A confirmed drop clears only stale state.
    if(execute && std::string(e.what()).find("OBJECT_DROPPED")!=std::string::npos){
      try {
        auto cleared=call<std_srvs::srv::Trigger>(node,"/mujoco/"+object+"/clear_dropped",std::make_shared<std_srvs::srv::Trigger::Request>());
        require(cleared->success,cleared->message);stabilized=false;
        auto pause_request=std::make_shared<std_srvs::srv::SetBool::Request>();pause_request->data=true;
        auto drained=call<std_srvs::srv::SetBool>(node,"/dynamic_object_scene_sync/pause",pause_request);paused=true;
        require(drained->success,"SCENE_SYNC_FAILED drop cleanup drain; reset required");
        geometry_msgs::msg::PoseStamped p;require(rclcpp::wait_for_message(p,reader,"/mujoco/objects/"+object+"/pose",2s),"OBJECT_NOT_FOUND drop pose");
        require(std::abs((node->now()-rclcpp::Time(p.header.stamp)).seconds())<=0.5,"SCENE_SYNC_FAILED stale drop pose");
        hm::ObjectRegistry registry(ament_index_cpp::get_package_share_directory("home_robotics_control")+"/config/objects.yaml");
        auto request=std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
        request->scene.is_diff=true;request->scene.robot_state.is_diff=true;
        moveit_msgs::msg::AttachedCollisionObject remove;remove.object.id=object;remove.object.operation=remove.object.REMOVE;
        request->scene.robot_state.attached_collision_objects.push_back(remove);
        request->scene.world.collision_objects.push_back(hm::world_object(registry.at(object),p));
        require(call<moveit_msgs::srv::ApplyPlanningScene>(node,"/apply_planning_scene",request)->success,"SCENE_SYNC_FAILED drop cleanup");attached=false;
      }catch(const std::exception& cleanup){report["cleanup_failure"]=std::string(cleanup.what());}
    }
    if(stage!=S::FAILED&&stage!=S::DONE)transition(S::FAILED,"FAILED");
    if(execute&&paused){
      try{auto q=std::make_shared<std_srvs::srv::SetBool::Request>();q->data=false;
        call<std_srvs::srv::SetBool>(node,"/dynamic_object_scene_sync/pause",q);paused=false;}
      catch(const std::exception& cleanup){RCLCPP_ERROR(log,"Scene sync resume failed: %s",cleanup.what());}
    }
    RCLCPP_ERROR(log,"DEMO FAILED: %s; stabilization=%d attached=%d sync_paused=%d. Supported release required; inspect/reset before retry.",e.what(),stabilized,attached,paused);
  }
  report["execute"]=execute;report["planning_time"]=planning_seconds;
  report["total_duration"]=std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();
  if(!result_file.empty()){std::ofstream output(result_file);output<<report; if(!output){RCLCPP_ERROR(log,"Cannot save result file");result=1;}}
  executor.cancel();spinner.join();rclcpp::shutdown();return result;
}
