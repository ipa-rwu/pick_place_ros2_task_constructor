#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/task.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>

#include <rclcpp/rclcpp.hpp>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

#include <pick_place_app/pick_place_task.hpp>

const rclcpp::Logger LOGGER = rclcpp::get_logger("pick_place_task");
namespace mtc = moveit::task_constructor;

void PickPlaceTask::Parameters::loadParameters(const rclcpp::Node::SharedPtr& node)
{
  // Critical parameters (no default exists => shutdown if loading fails)
  size_t errors = 0;
  errors += !rosparam_shortcuts::get(node, "arm_group_name",
                                     arm_group_name);  // e.g. panda_arm

  errors += !rosparam_shortcuts::get(node, "end_effector_name",
                                     end_effector_name);  // e.g. == hand_group_name, hand
  errors += !rosparam_shortcuts::get(node, "hand_frame",
                                     hand_frame);  // e.g. panda_hand
  errors += !rosparam_shortcuts::get(node, "object_pose", object_pose);
  errors += !rosparam_shortcuts::get(node, "place_pose", place_pose);
  errors += !rosparam_shortcuts::get(node, "object_frame_id", object_frame_id);
  errors += !rosparam_shortcuts::get(node, "place_frame_id", place_frame_id);

  errors += !rosparam_shortcuts::get(node, "object_name", object_name);

  errors += !rosparam_shortcuts::get(node, "gripper_io_server",
                                     gripper_io_server_name);  // e.g. panda_arm

  rosparam_shortcuts::shutdownIfError(errors);

  // Optional parameters (default value exists => no shutdown required if
  // loading fails)
  size_t warnings = 0;
  warnings += !rosparam_shortcuts::get(node, "hand_group_name",
                                       hand_group_name);  // e.g. hand
  warnings += !rosparam_shortcuts::get(node, "hand_open_pose",
                                       hand_open_pose);  // e.g. open
  warnings += !rosparam_shortcuts::get(node, "hand_close_pose",
                                       hand_close_pose);  // e.g. close
  RCLCPP_WARN(LOGGER, "Failed to load optional parameters.");
}

PickPlaceTask::PickPlaceTask(const rclcpp::Node::SharedPtr& node, const PickPlaceTask::Parameters& parameters)
{
  using namespace moveit::task_constructor;
  RCLCPP_INFO(LOGGER, "Initializing task pipeline");
  task_ = std::make_unique<mtc::Task>();  // pick_place_task
  task_->loadRobotModel(node);

  task_->setProperty("group", parameters.arm_group_name);
  task_->setProperty("eef", parameters.hand_group_name);
  task_->setProperty("ik_frame", parameters.hand_frame);

  /**************************
   * set up planner *
   **************************/
  // PipelinePlanner uses MoveIt’s planning pipeline, which typically defaults
  // to OMPL.
  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node);

  // JointInterpolation is a simple planner that interpolates between the start
  // and goal joint states. It is typically used for simple motions as it
  // computes quickly but doesn’t support complex motions
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  // CartesianPath is used to move the end effector in a straight line in
  // Cartesian space.
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScaling(1.0);
  cartesian_planner->setMaxAccelerationScaling(1.0);
  cartesian_planner->setStepSize(.01);

  /**************************
   * Current State *
   **************************/
  mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
  {
    auto current_state = std::make_unique<mtc::stages::CurrentState>("current state");
    current_state_ptr = current_state.get();
    task_->add(std::move(current_state));
  }

  // TODO: use io
  /**************************
   * Open Hand *
   **************************/
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", sampling_planner);
    stage->setGroup(parameters.hand_group_name);
    stage->setGoal(parameters.hand_open_pose);
    task_->add(std::move(stage));
  }

  // {
  //   auto stage = std::make_unique<mtc::stages::UseIOGripper>("open hand", parameters.gripper_io_server_name);
  //   stage->setGripperState("open");
  //   task_->add(std::move(stage));
  // }

  /**************************
   * Move To Object's Pose *
   **************************/
  // move the arm to a position where we can pick up our object. This is done
  // with a Connect stage, which as its name implies, is a Connector stage. This
  // means that it tries to bridge between the results of the stage before and
  // after it.
  {
    auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
        "move to pick", mtc::stages::Connect::GroupPlannerVector{ { parameters.arm_group_name, sampling_planner } });
    stage_move_to_pick->setTimeout(5.0);
    stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
    task_->add(std::move(stage_move_to_pick));
  }

  mtc::Stage* attach_object_stage = nullptr;  // Forward attach_object_stage to place pose generator

  // This is an example of SerialContainer usage. It's not strictly needed here.
  // In fact, `task` itself is a SerialContainer by default.
  {
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    task_->properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
    grasp->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", parameters.hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.1, 0.15);

      // Set hand forward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = parameters.hand_frame;
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    /** Generate Grasp Pose **/
    {
      // Sample grasp pose
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");
      stage->setPreGraspPose("open");
      stage->setObject(parameters.object_name);
      stage->setAngleDelta(M_PI / 12);
      // Hook into current state
      stage->setMonitoredStage(current_state_ptr);

      // This is the transform from the object frame to the end-effector frame
      Eigen::Isometry3d grasp_frame_transform;
      Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
                             Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
      grasp_frame_transform.linear() = q.matrix();
      grasp_frame_transform.translation().z() = 0.1;

      // Compute IK
      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(grasp_frame_transform, parameters.hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      grasp->insert(std::move(wrapper));
    }

    /** Allow Collision (hand object) **/
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
      stage->allowCollisions("object",
                             task_->getRobotModel()
                                 ->getJointModelGroup(parameters.hand_group_name)
                                 ->getLinkModelNamesWithCollisionGeometry(),
                             true);
      grasp->insert(std::move(stage));
    }

    /** Close hand **/
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
      stage->setGroup(parameters.hand_group_name);
      stage->setGoal("close");
      grasp->insert(std::move(stage));
    }

    /** Attach object **/
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject("object", parameters.hand_frame);
      attach_object_stage = stage.get();
      grasp->insert(std::move(stage));
    }

    /** Lift object **/
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.1, 0.3);
      stage->setIKFrame(parameters.hand_frame);
      stage->properties().set("marker_ns", "lift_object");

      // Set upward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    task_->add(std::move(grasp));
  }

  /**************************
   * Move to Place *
   **************************/
  {
    auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
        "move to place", mtc::stages::Connect::GroupPlannerVector{ { parameters.arm_group_name, sampling_planner },
                                                                   { parameters.hand_group_name, sampling_planner } });
    stage_move_to_place->setTimeout(5.0);
    stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
    task_->add(std::move(stage_move_to_place));
  }

  /**************************
   * Place Object *
   **************************/
  {
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    task_->properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
    place->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

    {
      // Sample place pose
      auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "place_pose");
      stage->setObject(parameters.object_name);

      geometry_msgs::msg::PoseStamped target_pose_msg;
      target_pose_msg.header.frame_id = parameters.place_frame_id;
      target_pose_msg.pose.position = parameters.place_pose.position;
      stage->setPose(target_pose_msg);
      stage->setMonitoredStage(attach_object_stage);  // Hook into attach_object_stage

      // Compute IK
      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(parameters.hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      place->insert(std::move(wrapper));
    }

    /** Open Hand **/
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
      stage->setGroup(parameters.hand_group_name);
      stage->setGoal(parameters.hand_open_pose);
      place->insert(std::move(stage));
    }

    /** Forbid collision(hand,object) **/
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
      stage->allowCollisions("object",
                             task_->getRobotModel()
                                 ->getJointModelGroup(parameters.hand_group_name)
                                 ->getLinkModelNamesWithCollisionGeometry(),
                             false);
      place->insert(std::move(stage));
    }

    /** Detach Object **/
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject("object", parameters.hand_frame);
      place->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.1, 0.3);
      stage->setIKFrame(parameters.hand_frame);
      stage->properties().set("marker_ns", "retreat");

      // Set retreat direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.x = -0.5;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }
    task_->add(std::move(place));
  }

  /**************************
   * Return home *
   **************************/

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setGoal("ready");
    task_->add(std::move(stage));
  }
}

bool PickPlaceTask::plan()
{
  RCLCPP_INFO(LOGGER, "Start searching for task solutions");
  task_->enableIntrospection();

  try
  {
    task_->init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return false;
  }

  if (!task_->plan(5 /* max_solutions */))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return false;
  }

  if (task_->numSolutions() == 0)
  {
    RCLCPP_ERROR(LOGGER, "Planning failed");
    return false;
  }
  return true;
}

bool PickPlaceTask::execute()
{
  RCLCPP_INFO(LOGGER, "Executing solution trajectory");

  task_->introspection().publishSolution(*task_->solutions().front());

  moveit_msgs::msg::MoveItErrorCodes execute_result = task_->execute(*task_->solutions().front());

  if (execute_result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed and returned: " << execute_result.val);
    return false;
  }

  return true;
}
