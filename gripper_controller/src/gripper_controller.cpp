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

#include <gripper_controller/gripper_controller.h>
#include <pluginlib/class_list_macros.h>
#include <angles/angles.h>
#include <math.h>

namespace gripper_controller
{

const std::string red("\033[0;31m");
const std::string green("\033[1;32m");
const std::string yellow("\033[1;33m");
const std::string cyan("\033[0;36m");
const std::string magenta("\033[0;35m");
const std::string reset("\033[0m");

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
  GripperController::GripperController() {}
  GripperController::~GripperController() {/*sub_command_.shutdown();*/}

  int computeFinalAngle(bool external_grip, float diameter)
  {
    // ============================ ONROBOT PARAMETERS DEFINITION
    int finger_position = 2; //possible values 1, 2 or 3
    int fingertip_length = 2; //possible values: 1(10mm), 2(13mm), 3(16.5mm)

    int ranges[3] = {165, 163, 161};
    int min_external_angle = ranges[finger_position - 1];
    int min_internal_angle = 160;
    int max_angle = 30;
    //columns:      ext_grip_min | ext_grip_max | int_grip_min | int_grip_max
    //rows:         pos 1: length 1 | 2 | 3 | pos 2: length 1 | 2 | 3 | pos 3: length | length 1 | 2 | 3
    int matrix[9][4] = {
            {10, 117, 35, 135},
            {7, 114, 38, 138},
            {4, 111, 41, 140},
            {26, 134, 49, 153},
            {23, 131, 52, 156},
            {20, 128, 55, 158},
            {44, 152, 65, 172},
            {41, 149, 68, 174},
            {38, 146, 71, 176}};

    int actual_min_diameter = matrix[(finger_position * 3) + fingertip_length - 4][(external_grip ? 0 : 2)];
    int actual_max_diameter = matrix[(finger_position * 3) + fingertip_length - 4][(external_grip ? 1 : 3)];
    // ============================ END


    // ============================ DIAMETER VALIDATION
    if(!(diameter >= actual_min_diameter && diameter <= actual_max_diameter)){
        ROS_ERROR("DIAMETER NOT VALID: OUT OF PERMITTED RANGE "/* + actual_min_diameter + "-" + actual_max_diameter + "FOR ACTUAL POSITION"*/);
        return false;
    }
    // ============================ END



    // ============================ FINAL ANGLE COMPUTATION
    int actual_min_angle    = external_grip ? min_external_angle : min_internal_angle;

    //diameter - actual_min_diameter : actual_max_diameter - actual_min_diameter = actual_min_angle - x : actual_min_angle - max_angle

    float final_angle = (actual_min_angle - max_angle) * (diameter - actual_min_diameter) / (actual_max_diameter - actual_min_diameter);
    final_angle = actual_min_angle - final_angle;

    /*std::cout << min_external_angle << std::endl;
    std::cout << actual_min_angle << std::endl;
    std::cout << actual_min_diameter << std::endl;
    std::cout << actual_max_diameter << std::endl;*/

    return final_angle;
    // ============================ END
  }

  int sendOnrobotScript(char* mode, std::vector<std::string> arguments)
  {
    // ============================ SENDING SCRIPT TO ONROBOT SOCKET
    char *open_script = "/scripts/gripper_open.py";
    char *close_script = "/scripts/gripper_close.py";
    char *script_name;
    if(mode == "open")
        script_name = open_script;
    else if(mode == "close")
        script_name = close_script;
    else{
        ROS_ERROR("CANNOT SEND ONROBOT SCRIPT: UNKNOWN MODE");
        return 1;
    }
    const char* script_path = (ros::package::getPath("gripper_controller").append(script_name)).data();

    FILE* fp;
	Py_Initialize();

    //CONVERTING ARGUMENTS TO UNICODE
    std::vector<wchar_t*> argv;
    for (const auto& arg : arguments){
        long unsigned int mySize = 200;
        wchar_t* wargv = Py_DecodeLocale(arg.data(), &mySize);
        argv.push_back(wargv);
    }
    argv.push_back(nullptr);
	PySys_SetArgv(argv.size()-1, &argv[0]);

	fp = _Py_fopen((ros::package::getPath("gripper_controller").append(script_name)).data(), "r");
	int result = PyRun_SimpleFile(fp, (ros::package::getPath("gripper_controller").append(script_name)).data());
	Py_Finalize();

	return result;
  }

  bool GripperController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
  {
    // List of controlled joints
    std::string param_name = "joints";
    if(!n.getParam(param_name, joint_names_))
    {
      ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << n.getNamespace() << ").");
      return false;
    }
    n_joints_ = joint_names_.size();

    if(n_joints_ == 0){
      ROS_ERROR_STREAM("List of joint names is empty.");
      return false;
    }

    // Get URDF
    urdf::Model urdf;
    if (!urdf.initParam("robot_description"))
    {
      ROS_ERROR("Failed to parse urdf file");
      return false;
    }

    pid_controllers_.resize(n_joints_);

    for(unsigned int i=0; i<n_joints_; i++)
    {
      try
      {
        joints_.push_back(hw->getHandle(joint_names_[i]));
      }
      catch (const hardware_interface::HardwareInterfaceException& e)
      {
        ROS_ERROR_STREAM("Exception thrown: " << e.what());
        return false;
      }

      urdf::JointConstSharedPtr joint_urdf = urdf.getJoint(joint_names_[i]);
      if (!joint_urdf)
      {
        ROS_ERROR("Could not find joint '%s' in urdf", joint_names_[i].c_str());
        return false;
      }
      joint_urdfs_.push_back(joint_urdf);

      // Load PID Controller using gains set on parameter server
      if (!pid_controllers_[i].init(ros::NodeHandle(n, joint_names_[i] + "/pid")))
      {
        ROS_ERROR_STREAM("Failed to load PID parameters from " << joint_names_[i] + "/pid");
        return false;
      }
    }

    commands_buffer_.writeFromNonRT(std::vector<double>(n_joints_, 0.0));

    //sub_command_ = n.subscribe<std_msgs::Float64>("command", 1, &GripperController::commandCB, this);
    std::string gripper_name = "gripper";
    std::string robot_name = "robot";
    ros::NodeHandle param_node;
    param_node.getParam("/robot_name", robot_name);
    param_node.getParam("/gripper_name", gripper_name);

    std::cout<< red<< "ROBOT NAME IS : "<< robot_name<<reset <<std::endl;
    std::cout<< red<< "GRIPPER NAME IS : "<< gripper_name<<reset <<std::endl;
    gripper_open_srv_  = param_node.advertiseService("/"+robot_name + "/"+gripper_name + "/open", &GripperController::gripperOpenCallback, this);
    gripper_close_srv_ = param_node.advertiseService("/"+robot_name + "/"+gripper_name + "/close", &GripperController::gripperCloseCallback, this);


    return true;
  }

  void GripperController::update(const ros::Time& time, const ros::Duration& period)
  {
    std::vector<double> & commands = *commands_buffer_.readFromRT();
    for(unsigned int i=0; i<n_joints_; i++)
    {
        double command_position = commands[i];

        double error; //, vel_error;
        double commanded_effort;

        double current_position = joints_[i].getPosition();

        // Make sure joint is within limits if applicable
        enforceJointLimits(command_position, i);

        // Compute position error
        if (joint_urdfs_[i]->type == urdf::Joint::REVOLUTE)
        {
         angles::shortest_angular_distance_with_limits(
            current_position,
            command_position,
            joint_urdfs_[i]->limits->lower,
            joint_urdfs_[i]->limits->upper,
            error);
        }
        else if (joint_urdfs_[i]->type == urdf::Joint::CONTINUOUS)
        {
          error = angles::shortest_angular_distance(current_position, command_position);
        }
        else //prismatic
        {
          error = command_position - current_position;
        }

        // Set the PID error and compute the PID command with nonuniform
        // time step size.
        commanded_effort = pid_controllers_[i].computeCommand(error, period);

        joints_[i].setCommand(commanded_effort);
    }
  }

  bool GripperController::gripperOpenCallback(gripper_open::Request& req, gripper_open::Response& res)
  {
    if(!req.diameter)
    {
      ROS_ERROR_STREAM("Command does not contain diameter value! Not executing!");
      return false;
    }
    int final_angle = computeFinalAngle(req.external_release, req.diameter);

    std::vector<std::string> arguments = { std::to_string(req.diameter),
            std::to_string(req.external_release), std::to_string(req.tool_index),
            std::to_string(req.select_releasing), std::to_string(req.blocking)};

    int result = sendOnrobotScript("open", arguments);
    if(result == 0)
	    res.ack = true;
	else
	    res.ack = false;

    // ============================ UPDATING JOINT GOALS
    std_msgs::Float64MultiArray command;
    command.data.resize(3);
    command.data[0] = final_angle * M_PI / 180;
    command.data[1] = final_angle * M_PI / 180;
    command.data[2] = final_angle * M_PI / 180;
    commands_buffer_.writeFromNonRT(command.data);
    // ============================ END

    return true;
  }

  bool GripperController::gripperCloseCallback(gripper_close::Request& req, gripper_close::Response& res)
  {
    if(!req.diameter)
    {
      ROS_ERROR_STREAM("Command does not contain diameter value! Not executing!");
      return false;
    }

    int final_angle = computeFinalAngle(req.external_grip, req.diameter);

    std::vector<std::string> arguments = { std::to_string(req.diameter), std::to_string(req.force),
        std::to_string(req.external_grip), std::to_string(req.stop_if_no_force),
        std::to_string(req.tool_index), std::to_string(req.blocking)};
    int result = sendOnrobotScript("close", arguments);
    if(result == 0)
	    res.ack = true;
	else
	    res.ack = false;

    // ============================ UPDATING JOINT GOALS
    std_msgs::Float64MultiArray command;
    command.data.resize(3);
    command.data[0] = final_angle * M_PI / 180;
    command.data[1] = final_angle * M_PI / 180;
    command.data[2] = final_angle * M_PI / 180;
    commands_buffer_.writeFromNonRT(command.data);
    // ============================ END

    return true;
  }

  void GripperController::enforceJointLimits(double &command, unsigned int index)
  {
    // Check that this joint has applicable limits
    if (joint_urdfs_[index]->type == urdf::Joint::REVOLUTE || joint_urdfs_[index]->type == urdf::Joint::PRISMATIC)
    {
      if( command > joint_urdfs_[index]->limits->upper ) // above upper limit
      {
        command = joint_urdfs_[index]->limits->upper;
      }
      else if( command < joint_urdfs_[index]->limits->lower ) // below lower limit
      {
        command = joint_urdfs_[index]->limits->lower;
      }
    }
  }

} // namespace

PLUGINLIB_EXPORT_CLASS( gripper_controller::GripperController, controller_interface::ControllerBase)