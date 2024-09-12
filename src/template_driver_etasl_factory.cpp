
#include "etasl_task_utils/etasl_error.hpp"
#include "robot_interfacing_utils/robotdriverfactory.hpp"
#include "etasl_task_utils/registry.hpp"
#include "robot_interfacing_utils/feedback_struct.hpp"
#include <jsoncpp/json/json.h>

#include "template_driver_etasl/template_driver_etasl.hpp"

#include <iostream>

namespace etasl {

/**
 * This is a factory that can create a TemplateDriverEtasl
 * The platform specific part is given with the constructor of this factory
 * Afterwards, everything is generic and independent of the platform
 */
class TemplateDriverEtaslFactory : public RobotDriverFactory {

    FeedbackMsg* feedback_ptr;
    SetpointMsg* setpoint_ptr; 

public:
    typedef std::shared_ptr<RobotDriverFactory> SharedPtr;

    TemplateDriverEtaslFactory(FeedbackMsg* _feedback_ptr, SetpointMsg* _setpoint_ptr)
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
                        "$schema": "http://json-schema.org/draft-04/schema",
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
                            },
                            "joint_names": {
                                "description":"Names of the joints in order from the world to the end effector. This names need to be consistent with the ones declared in the URDF file of the robot.",
                                "type":"array",
                                "items": {"type": "string"},
                                "default": ["joint1", "joint2", "joint3", "etc..."]
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
     * @brief gets the name of this solver
     */
    virtual const char* getName()
    {
        return "template_driver_etasl";
    }

    /**
     * @brief create the solver with the given parameters
     *
     */
    virtual RobotDriver::SharedPtr create(const Json::Value& parameters)
    {
        std::string p_ip_address = parameters["ip_address"].asString();
        
        // get fri_port from parameters
        unsigned int p_fri_port = parameters["fri_port"].asUInt();

        // print fri_port
        std::cout << "------IP: " << p_ip_address << "  ,   fri_port: " << p_fri_port << " ,   type:" << parameters["fri_port"].isNull() << std::endl;



        // std::vector<double> init_joints;
        // // init_joints.resize(parameters["initial_joints"].size(), 0.0);
        // for (auto n : parameters["initial_joints"]) {
        //     init_joints.push_back(n.asDouble());
        // }
        std::string name = getName();

        // for (auto n : parameters["variable-names"]) {
        //     varnames.push_back(n.asString());
        // }

        // TemplateDriverEtasl::TemplateDriverEtasl(
        //     std::string robot_name,
        //     FeedbackMsg* fb,
        //     SetpointMsg* sp,
        //     double periodicity_val,
        //     std::vector<double> init_joints)

        auto shared_robot_driv =  std::make_shared<TemplateDriverEtasl>();

        shared_robot_driv->construct(name, feedback_ptr, setpoint_ptr, parameters);



        return shared_robot_driv;

        //         return std::make_shared<TemplateDriverEtasl>(
        // name, 
        // feedback_ptr, 
        // setpoint_ptr, 
        // p_ip_address, 
        // p_fri_port);
    }

    virtual ~TemplateDriverEtaslFactory() { }
};

void registerTemplateDriverEtaslFactory(FeedbackMsg* _feedback_ptr, SetpointMsg* _setpoint_ptr)
{
    // be sure to use the BASE CLASS as template parameter for the Registry!
    Registry<RobotDriverFactory>::registerFactory(std::make_shared<TemplateDriverEtaslFactory>(_feedback_ptr, _setpoint_ptr));
}

} // namespace