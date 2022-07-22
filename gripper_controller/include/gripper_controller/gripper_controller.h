/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  Copyright (c) 2012, hiDOF, Inc.
 *  Copyright (c) 2013, PAL Robotics, S.L.
 *  Copyright (c) 2014, Fraunhofer IPA
 *  All rights reserved.
 *
 *********************************************************************/

#ifndef EFFORT_CONTROLLERS_JOINT_GROUP_POSITION_CONTROLLER_H
#define EFFORT_CONTROLLERS_JOINT_GROUP_POSITION_CONTROLLER_H

#include <ros/node_handle.h>
#include <urdf/model.h>
#include <control_toolbox/pid.h>
#include <boost/scoped_ptr.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <control_msgs/JointControllerState.h>
#include <realtime_tools/realtime_buffer.h>

#include <gripper_controller/gripper_open.h>
#include <gripper_controller/gripper_close.h>

#include <ros/package.h>
#include <Python.h>

namespace gripper_controller
{

/**
 * \brief Forward command controller for a set of effort controlled joints (torque or force).
 *
 * This class forwards the commanded efforts down to a set of joints.
 *
 * \section ROS interface
 *
 * \param type Must be "JointGroupEffortController".
 * \param joints List of names of the joints to control.
 *
 * Subscribes to:
 * - \b command (std_msgs::Float64MultiArray) : The joint efforts to apply
 */
class GripperController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
  GripperController();
  ~GripperController();

  bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n);
  void update(const ros::Time& /*time*/, const ros::Duration& /*period*/);

  std::vector< std::string > joint_names_;
  std::vector< hardware_interface::JointHandle > joints_;
  realtime_tools::RealtimeBuffer<std::vector<double> > commands_buffer_;
  unsigned int n_joints_;

private:

  std::vector<control_toolbox::Pid> pid_controllers_;       /**< Internal PID controllers. */

  std::vector<urdf::JointConstSharedPtr> joint_urdfs_;

  ros::ServiceServer gripper_open_srv_;
  ros::ServiceServer gripper_close_srv_;

  bool gripperOpenCallback(gripper_open::Request& req, gripper_open::Response& res);
  bool gripperCloseCallback(gripper_close::Request& req, gripper_close::Response& res);
  void enforceJointLimits(double &command, unsigned int index);
}; // class

} // namespace

#endif