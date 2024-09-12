#pragma once

#include "robot_interfacing_utils/feedback_struct.hpp"

namespace etasl {

void registerTemplateDriverEtaslFactory(FeedbackMsg* _feedback_ptr, SetpointMsg* _setpoint_ptr);


} // namespace
