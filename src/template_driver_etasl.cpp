#include "template_driver_etasl/template_driver_etasl.hpp"
#include <fmt/format.h>
#include <iostream>

// includ library for sleep
#include <chrono>
#include <thread>

namespace etasl {


template_driver_etasl::template_driver_etasl()
{
    setpoint_joint_vel_.data.resize(DOF, 0.0); //Initialize setpoint joint velocities to zero
}

void template_driver_etasl::construct(std::string robot_name, 
                        const Json::Value& config,
                        std::shared_ptr<etasl::JsonChecker> jsonchecker)
{

    periodicity_ = jsonchecker->asDouble(config, "periodicity");
    name = robot_name; //defined in RobotDriver super class.


    AvailableFeedback available_fb{};
    available_fb.joint_pos = true;
    // Uncomment the following lines to enable more feedback variables, if your robot supports them.
    // available_fb.joint_vel = true;
    // available_fb.joint_torque= true;
    // available_fb.joint_current= true;
    // available_fb.cartesian_pos= true;
    // available_fb.cartesian_quat= true;
    // available_fb.cartesian_twist= true;
    // available_fb.cartesian_wrench= true;
    // available_fb.base_pos= true;
    // available_fb.base_quat= true;
    // available_fb.base_twist= true;

    constructPorts(DOF, available_fb); //Constructs all shared pointers and initialize data structures. Call after assigning available_feedback booleans.

    std::cout << "Constructed robot driver with name: " << name << std::endl;

}

bool template_driver_etasl::initialize()
{
    //READ INITIAL JOINT POSITIONS FROM THE ROBOT AND SAVE THEM IN initial_joints

    std::vector<float> initial_joints = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; //REPLACE with actual reading of initial joint positions from the robot
    std::copy(initial_joints.begin(), initial_joints.end(), joint_pos_.data.begin()); //joint_pos_ = initial_joints;
    writeFeedbackJointPosition(joint_pos_);

    return true;
}


void template_driver_etasl::update(volatile std::atomic<bool>& stopFlag)
{
    readSetpointJointVelocity(setpoint_joint_vel_);

    assert(joint_pos_.data.size() == setpoint_joint_vel_.data.size());

    for (unsigned int i=0; i<setpoint_joint_vel_.data.size(); ++i) {
        joint_pos_.data[i] += setpoint_joint_vel_.data[i]*periodicity_; //EXAMPLE: simple integration (REPLACE with your robot control command)
    }

    writeFeedbackJointPosition(joint_pos_);
}

void template_driver_etasl::on_configure() {
    // std::cout << "entering on configure =======================" << std::endl;

}

void template_driver_etasl::on_activate() 
{


}

void template_driver_etasl::on_deactivate() {
    // std::cout << "entering on deactivate =======================" << std::endl;

}

void template_driver_etasl::on_cleanup() {
    // std::cout << "entering on cleanup =======================" << std::endl;

}


void template_driver_etasl::finalize() {
    std::cout << "finalize() called =======================" << std::endl;

    //REPLACE WITH INSTRUCTIONS TO SAFELY FINALIZE COMMUNICATION WITH THE ROBOT
    // THIS WILL ONLY BE CALLED ONCE WHEN THE CONTROLLER SHUTS DOWN

}



template_driver_etasl::~template_driver_etasl() {

};



} // namespace etasl


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(etasl::template_driver_etasl, etasl::RobotDriver)
