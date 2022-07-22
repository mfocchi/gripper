/**
 * @file gripper_controller.cpp
 * @author Simone Brentan
 * @date 20 June, 2022
 * @brief Gripper controller.
 */

#include <gripper_controller/gripper_controller.h>


namespace gripper_controller {

const std::string red("\033[0;31m");
const std::string green("\033[1;32m");
const std::string yellow("\033[1;33m");
const std::string cyan("\033[0;36m");
const std::string magenta("\033[0;35m");
const std::string reset("\033[0m");

GripperController::GripperController()
{
}

GripperController::~GripperController()
{

}

bool GripperController::init(hardware_interface::RobotHW* robot_hw,
                      ros::NodeHandle& root_nh,
                      ros::NodeHandle& controller_nh)
{
    // getting the names of the joints from the ROS parameter server
    std::cout<< red<< "Initialize Gripper Controller" << reset <<std::endl;
    root_nh_ = &root_nh;
    assert(robot_hw);

    hardware_interface::EffortJointInterface* eff_hw = robot_hw->get<hardware_interface::EffortJointInterface>();

    if(!eff_hw)
    {
        ROS_ERROR("hardware_interface::EffortJointInterface not found");
        return false;
    }

    if (!controller_nh.getParam("joint", joint_name_))
    {
        ROS_ERROR("No joint given in the namespace: %s.", controller_nh.getNamespace().c_str());
        return false;
    }

    if (!controller_nh.getParam("gain", joint_gain_))
    {
        ROS_ERROR("Couldn't find the gain parameter value");
        return false;
    }


    // Getting joint state handle
    try
    {
        std::cout<< green<< "Loading effort interface for " <<joint_name_<< reset <<std::endl;
        joint_handle_ = eff_hw->getHandle(joint_name_);

    }
    catch(...)
    {
        ROS_ERROR("Error loading the effort interfaces");
        return false;
    }
    //assert(joint_handle_);

    std::string gripper_name = "gripper";
    std::string robot_name = "robot";
    ros::NodeHandle param_node;
    param_node.getParam("/robot_name", robot_name);
    param_node.getParam("/gripper_name", gripper_name);

    std::cout<< red<< "ROBOT NAME IS : "<< robot_name<<reset <<std::endl;
    std::cout<< red<< "GRIPPER NAME IS : "<< gripper_name<<reset <<std::endl;

    //sub_ = root_nh.subscribe<gripper_controller::GripperState>("/"+robot_name + "/"+gripper_name + "/command", 1, &GripperController::commandCallback, this);
    //sub_ = root_nh.subscribe("command", 1, &GripperController::commandCallback, this);

    gripper_open_srv_  = param_node.advertiseService("/"+robot_name + "/"+gripper_name + "/open", &GripperController::gripperOpenCallback, this);
    gripper_close_srv_ = param_node.advertiseService("/"+robot_name + "/"+gripper_name + "/close", &GripperController::gripperCloseCallback, this);

    return true;
}


void GripperController::starting(const ros::Time& time)
{
    ROS_INFO("Starting Controller");
    joint_diameter_ = 0;
}


bool GripperController::gripperOpenCallback(gripper_open::Request& req, gripper_open::Response& res)
{
    ROS_INFO("GRIPPER OPEN");
    joint_diameter_ = 0.1;

    char script_name[] = "/scripts/gripper_open.py";
    const char* script_path = (ros::package::getPath("gripper_controller").append(script_name)).data();

    FILE* fp;
	Py_Initialize();

    //CONVERTING ARGUMENTS TO UNICODE
    std::vector<std::string> arguments = {
            std::to_string(req.diameter), std::to_string(req.force),
            std::to_string(req.external_grip), std::to_string(req.stop_if_no_force),
            std::to_string(req.tool_index), std::to_string(req.blocking)};
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

    if(result == 0)
	    res.ack = true;
	else
	    res.ack = false;

    return true;
}


bool GripperController::gripperCloseCallback(gripper_close::Request& req, gripper_close::Response& res)
{
    res.ack = true;
    ROS_INFO("GRIPPER CLOSE");
    joint_diameter_ = 0.05;

    char script_name[] = "/scripts/gripper_close.py";
    const char* script_path = (ros::package::getPath("gripper_controller").append(script_name)).data();

    FILE* fp;
	Py_Initialize();

    //CONVERTING ARGUMENTS TO UNICODE
    std::vector<std::string> arguments = {
            /*std::to_string(), */std::to_string(req.diameter),
            std::to_string(req.external_release), std::to_string(req.tool_index),
            std::to_string(req.select_releasing), std::to_string(req.blocking)};
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

	if(result == 0)
	    res.ack = true;
	else
	    res.ack = false;

    return true;
}


/*void GripperController::commandCallback(const gripper_controller::GripperState& msg)
{
     //TODO set joint_diameter = msg->diameter
     joint_diameter_ = msg.diameter;


}*/

void GripperController::update(const ros::Time& time, const ros::Duration& period)
{
    /*TODO
        double error = joint_diameter - joint_handle_.getPosition();
        double commanded_effort = error - joint_gain_;
        joint_handle_.setCommand(commanded_effort);
    */

    double error = joint_diameter_ - joint_handle_.getPosition();
    double commanded_effort = error * joint_gain_;
    joint_handle_.setCommand(commanded_effort);

    std::cout << red << commanded_effort << reset << std::endl;
}



void GripperController::stopping(const ros::Time& time)
{
    ROS_DEBUG("Stopping Controller");
}



} //namespace
