#include "template_driver_etasl/template_driver_etasl.hpp"
#include <fmt/format.h>
#include <iostream>

// includ library for sleep
#include <chrono>
#include <thread>

namespace etasl {


TemplateDriverEtasl::TemplateDriverEtasl()
    // : periodicity(periodicity_val)
    // , initial_joints(init_joints)
    // , joint_pos(init_joints)
{
}

void TemplateDriverEtasl::construct(std::string robot_name, 
                        FeedbackMsg* fb, 
                        SetpointMsg* sp,
                        const Json::Value& config,
                        boost::shared_ptr<etasl::JsonChecker> jsonchecker)
{

    periodicity = jsonchecker->asDouble(config, "periodicity");

    std::vector<double> init_joints;
    // init_joints.resize(parameters["initial_joints"].size(), 0.0);
    for (auto n : jsonchecker->asArray(config, "initial_joints")) {
        init_joints.push_back(jsonchecker->asDouble(n, ""));
    }

    initial_joints = init_joints;
    joint_pos = initial_joints;

    feedback_ptr = fb; //defined in RobotDriver super class.
    setpoint_ptr = sp; //defined in RobotDriver super class.
    name = robot_name; //defined in RobotDriver super class.
    std::cout << "Constructed object of TemplateDriverEtasl class with name: " << name << std::endl;

}

bool TemplateDriverEtasl::initialize()
{
    joint_pos = initial_joints;

    feedback_ptr->mtx.lock();
    setpoint_ptr->mtx.lock();

    feedback_ptr->joint.pos.data = joint_pos;
    feedback_ptr->joint.pos.is_available = true;

    setpoint_ptr->mtx.unlock();
    feedback_ptr->mtx.unlock();

    return true;
}


void TemplateDriverEtasl::update(volatile std::atomic<bool>& stopFlag)
{
    feedback_ptr->mtx.lock();
    setpoint_ptr->mtx.lock();

    assert(feedback_ptr->joint.pos.data.size() == setpoint_ptr->velocity.data.size());

    for (unsigned int i=0; i<feedback_ptr->joint.pos.data.size(); ++i) {
        joint_pos[i] += setpoint_ptr->velocity.data[i]*periodicity; //simple integration
        // joint_pos[i] += setpoint_ptr->velocity.data[i]*0.0000005; //simple integration
        // joint_pos[i] += 0.00000001; //test
        feedback_ptr->joint.pos.data[i] = joint_pos[i];
    }

    setpoint_ptr->velocity.fs = etasl::OldData;
    // std::cout << "vel val:" << setpoint_ptr->velocity.data[0] << " , " << setpoint_ptr->velocity.data[1] << " , "<< setpoint_ptr->velocity.data[2] << std::endl;


    // std::cout << "Driver update has set all pos values to " << feedback_ptr->joint.pos.data[0] << std::endl;
    // std::cout << "Driver update has set all pos values to " << this->periodicity << std::endl;
    // std::cout << "Driver update has set all pos values to " << getName() << std::endl;

    setpoint_ptr->mtx.unlock();
    feedback_ptr->mtx.unlock();
}

void TemplateDriverEtasl::on_configure() {
    // std::cout << "entering on configure =======================" << std::endl;

}

void TemplateDriverEtasl::on_activate() 
{


}

void TemplateDriverEtasl::on_deactivate() {
    // std::cout << "entering on deactivate =======================" << std::endl;

}

void TemplateDriverEtasl::on_cleanup() {
    // std::cout << "entering on cleanup =======================" << std::endl;

}


void TemplateDriverEtasl::finalize() {
    std::cout << "finalize() called =======================" << std::endl;

}



TemplateDriverEtasl::~TemplateDriverEtasl() {

};



} // namespace etasl


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(etasl::TemplateDriverEtasl, etasl::RobotDriver)
