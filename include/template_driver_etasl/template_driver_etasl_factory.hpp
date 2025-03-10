#pragma once

#include "robot_interfacing_utils/feedback_struct.hpp"

namespace etasl {

void register_template_driver_etasl_factory(FeedbackMsg* _feedback_ptr, SetpointMsg* _setpoint_ptr);


} // namespace
