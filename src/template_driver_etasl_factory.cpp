
#include "etasl_task_utils/etasl_error.hpp"
#include "robot_interfacing_utils/robotdriverfactory.hpp"
#include "etasl_task_utils/registry.hpp"
#include "robot_interfacing_utils/feedback_struct.hpp"
#include <jsoncpp/json/json.h>

#include "template_driver_etasl/template_driver_etasl.hpp"

#include <iostream>

namespace etasl {

/**
 * This is a factory that can create a template_driver_etasl
 * The platform specific part is given with the constructor of this factory
 * Afterwards, everything is generic and independent of the platform
 */
class template_driver_etasl_factory : public RobotDriverFactory {

    FeedbackMsg* feedback_ptr;
    SetpointMsg* setpoint_ptr; 

public:
    typedef std::shared_ptr<RobotDriverFactory> SharedPtr;

    template_driver_etasl_factory(FeedbackMsg* _feedback_ptr, SetpointMsg* _setpoint_ptr)
    :feedback_ptr(_feedback_ptr)
    ,setpoint_ptr(_setpoint_ptr)
    {
    }

    /**
     * @brief gets the schema for the parameters of this factory
     * @return JSON schema
     */
    virtual Json::Value getSchema()
    {
        std::string schema_src = R"(
                    {
                        "$schema": "http://json-schema.org/draft-06/schema",
                        "$id":"template_driver_etasl.json",
                        "type":"object",
                        "properties":{
                            "is-template_driver_etasl" : {
                                "description":"To indicate that the task will be executed in simulation, with a virtual robot that integrates joint velocities and gives back joint positions to eTaSL.",
                                "type":"boolean",
                                "default":true
                            },
                            "initial_joints" : {
                                "description":"Initial joint values in [rad] for rotational joints or [m] for prismatic joints. The size must coincide with the number of robot variables in eTaSL.",
                                "type":"array",
                                "items": {"type": "number"},
                                "default": [0.0, 0.0 ,0.0 ,0.0 ,0.0 ,0.0]
                            },
                            "periodicity" : {
                                "description":"Periodicity in which the integration will be performed in seconds. 1/periodicity is the update frequency in Hz.",
                                "type":"number",
                                "default": 0.01
                            }
                        },
                        "required":["is-template_driver_etasl","initial_joints", "periodicity"],
                        "additionalProperties": false
                )";
        Json::Value schema;
        Json::Reader reader;
        reader.parse(schema_src, schema);
        return schema;
    }

    /**
     * @brief gets the name of this driver
     */
    virtual const char* getName()
    {
        return "template_driver_etasl";
    }

    /**
     * @brief create the driver with the given parameters
     *
     */
    virtual RobotDriver::SharedPtr create(const Json::Value& parameters, boost::shared_ptr<etasl::JsonChecker> jsonchecker)
    {
        std::string p_ip_address = jsonchecker->asString(parameters, "ip_address");
        
        // get fri_port from parameters
        unsigned int p_fri_port = jsonchecker->asUInt(parameters, "fri_port");

        // print fri_port
        // std::cout << "------IP: " << p_ip_address << "  ,   fri_port: " << p_fri_port << " ,   type:" << parameters["fri_port"].isNull() << std::endl;



        // std::vector<double> init_joints;
        // // init_joints.resize(parameters["initial_joints"].size(), 0.0);
        // for (auto n : parameters["initial_joints"]) {
        //     init_joints.push_back(n.asDouble());
        // }
        std::string name = getName();

        // for (auto n : parameters["variable-names"]) {
        //     varnames.push_back(n.asString());
        // }

        // template_driver_etasl::template_driver_etasl(
        //     std::string robot_name,
        //     FeedbackMsg* fb,
        //     SetpointMsg* sp,
        //     double periodicity_val,
        //     std::vector<double> init_joints)

        auto shared_robot_driv =  std::make_shared<template_driver_etasl>();

        shared_robot_driv->construct(name, feedback_ptr, setpoint_ptr, parameters, jsonchecker);



        return shared_robot_driv;

        //         return std::make_shared<template_driver_etasl>(
        // name, 
        // feedback_ptr, 
        // setpoint_ptr, 
        // p_ip_address, 
        // p_fri_port);
    }

    virtual ~template_driver_etasl_factory() { }
};

void register_template_driver_etasl_factory(FeedbackMsg* _feedback_ptr, SetpointMsg* _setpoint_ptr)
{
    // be sure to use the BASE CLASS as template parameter for the Registry!
    Registry<RobotDriverFactory>::registerFactory(std::make_shared<template_driver_etasl_factory>(_feedback_ptr, _setpoint_ptr));
}

} // namespace