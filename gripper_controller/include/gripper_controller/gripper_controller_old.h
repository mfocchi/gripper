#ifndef GRIPPER_CONTROLLER_H
#define GRIPPER_CONTROLLER_H

// Ros
#include <ros/ros.h>
#include <gripper_controller/gripper_open.h>
#include <gripper_controller/gripper_close.h>
// PluginLib
#include <pluginlib/class_list_macros.hpp>
// Ros control
#include <controller_interface/controller.h>
#include <controller_interface/multi_interface_controller.h>
// Hardware interfaces
#include <hardware_interface/joint_command_interface.h>


#include <ros/package.h>
#include <Python.h>


namespace gripper_controller
{

class GripperController : public controller_interface::MultiInterfaceController<hardware_interface::EffortJointInterface>
{
public:
    /** @brief Constructor function */
    GripperController();

    /** @brief Destructor function */
    ~GripperController();

    /**
         * @brief Initializes sample controller
         * @param hardware_interface::RobotHW* robot hardware interface
         * @param ros::NodeHandle& Root node handle
         */
    bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);

    /**
         * @brief Starts the sample controller when controller manager request it
         * @param const ros::Time& time Time
         */
    void starting(const ros::Time& time);

    /**
         * @brief Updates the sample controller according to the control
         * frequency (task frequency)
         * @param const ros::Time& time Time
         * @param const ros::Duration& Period
         */
    void update(const ros::Time& time, const ros::Duration& period);

    /**
         * @brief Stops the sample controller when controller manager request it
         * @param const ros::time& Time
         */
    void stopping(const ros::Time& time);

private:

    //void GripperController::commandCallback(const float msg);

    bool gripperOpenCallback(gripper_open::Request& req, gripper_open::Response& res);
    bool gripperCloseCallback(gripper_close::Request& req, gripper_close::Response& res);

    ros::Subscriber sub_;
    std::string joint_name_;
    double joint_gain_;
    float joint_diameter_;
    hardware_interface::JointHandle joint_handle_;

    ros::ServiceServer gripper_open_srv_;
    ros::ServiceServer gripper_close_srv_;

    ros::NodeHandle * root_nh_;
    bool verbose = false;
    //dls::perception::GridMapTerrainROS grid_map_terrain_;
};


PLUGINLIB_EXPORT_CLASS(gripper_controller::GripperController, controller_interface::ControllerBase);

} //@namespace gripper_controller

#endif //GRIPPER_CONTROLLER_H
