// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_interfaces:action/Move.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__ACTION__DETAIL__MOVE__BUILDER_HPP_
#define CUSTOM_INTERFACES__ACTION__DETAIL__MOVE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_interfaces/action/detail/move__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_interfaces
{

namespace action
{

namespace builder
{

class Init_Move_Goal_time_goal
{
public:
  explicit Init_Move_Goal_time_goal(::custom_interfaces::action::Move_Goal & msg)
  : msg_(msg)
  {}
  ::custom_interfaces::action::Move_Goal time_goal(::custom_interfaces::action::Move_Goal::_time_goal_type arg)
  {
    msg_.time_goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::action::Move_Goal msg_;
};

class Init_Move_Goal_move_goal_ang_z
{
public:
  explicit Init_Move_Goal_move_goal_ang_z(::custom_interfaces::action::Move_Goal & msg)
  : msg_(msg)
  {}
  Init_Move_Goal_time_goal move_goal_ang_z(::custom_interfaces::action::Move_Goal::_move_goal_ang_z_type arg)
  {
    msg_.move_goal_ang_z = std::move(arg);
    return Init_Move_Goal_time_goal(msg_);
  }

private:
  ::custom_interfaces::action::Move_Goal msg_;
};

class Init_Move_Goal_move_goal_lin_x
{
public:
  explicit Init_Move_Goal_move_goal_lin_x(::custom_interfaces::action::Move_Goal & msg)
  : msg_(msg)
  {}
  Init_Move_Goal_move_goal_ang_z move_goal_lin_x(::custom_interfaces::action::Move_Goal::_move_goal_lin_x_type arg)
  {
    msg_.move_goal_lin_x = std::move(arg);
    return Init_Move_Goal_move_goal_ang_z(msg_);
  }

private:
  ::custom_interfaces::action::Move_Goal msg_;
};

class Init_Move_Goal_move_goal
{
public:
  Init_Move_Goal_move_goal()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Move_Goal_move_goal_lin_x move_goal(::custom_interfaces::action::Move_Goal::_move_goal_type arg)
  {
    msg_.move_goal = std::move(arg);
    return Init_Move_Goal_move_goal_lin_x(msg_);
  }

private:
  ::custom_interfaces::action::Move_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::action::Move_Goal>()
{
  return custom_interfaces::action::builder::Init_Move_Goal_move_goal();
}

}  // namespace custom_interfaces


namespace custom_interfaces
{

namespace action
{

namespace builder
{

class Init_Move_Result_result
{
public:
  Init_Move_Result_result()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::custom_interfaces::action::Move_Result result(::custom_interfaces::action::Move_Result::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::action::Move_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::action::Move_Result>()
{
  return custom_interfaces::action::builder::Init_Move_Result_result();
}

}  // namespace custom_interfaces


namespace custom_interfaces
{

namespace action
{

namespace builder
{

class Init_Move_Feedback_feedback
{
public:
  Init_Move_Feedback_feedback()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::custom_interfaces::action::Move_Feedback feedback(::custom_interfaces::action::Move_Feedback::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::action::Move_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::action::Move_Feedback>()
{
  return custom_interfaces::action::builder::Init_Move_Feedback_feedback();
}

}  // namespace custom_interfaces


namespace custom_interfaces
{

namespace action
{

namespace builder
{

class Init_Move_SendGoal_Request_goal
{
public:
  explicit Init_Move_SendGoal_Request_goal(::custom_interfaces::action::Move_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::custom_interfaces::action::Move_SendGoal_Request goal(::custom_interfaces::action::Move_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::action::Move_SendGoal_Request msg_;
};

class Init_Move_SendGoal_Request_goal_id
{
public:
  Init_Move_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Move_SendGoal_Request_goal goal_id(::custom_interfaces::action::Move_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_Move_SendGoal_Request_goal(msg_);
  }

private:
  ::custom_interfaces::action::Move_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::action::Move_SendGoal_Request>()
{
  return custom_interfaces::action::builder::Init_Move_SendGoal_Request_goal_id();
}

}  // namespace custom_interfaces


namespace custom_interfaces
{

namespace action
{

namespace builder
{

class Init_Move_SendGoal_Response_stamp
{
public:
  explicit Init_Move_SendGoal_Response_stamp(::custom_interfaces::action::Move_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::custom_interfaces::action::Move_SendGoal_Response stamp(::custom_interfaces::action::Move_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::action::Move_SendGoal_Response msg_;
};

class Init_Move_SendGoal_Response_accepted
{
public:
  Init_Move_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Move_SendGoal_Response_stamp accepted(::custom_interfaces::action::Move_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_Move_SendGoal_Response_stamp(msg_);
  }

private:
  ::custom_interfaces::action::Move_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::action::Move_SendGoal_Response>()
{
  return custom_interfaces::action::builder::Init_Move_SendGoal_Response_accepted();
}

}  // namespace custom_interfaces


namespace custom_interfaces
{

namespace action
{

namespace builder
{

class Init_Move_GetResult_Request_goal_id
{
public:
  Init_Move_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::custom_interfaces::action::Move_GetResult_Request goal_id(::custom_interfaces::action::Move_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::action::Move_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::action::Move_GetResult_Request>()
{
  return custom_interfaces::action::builder::Init_Move_GetResult_Request_goal_id();
}

}  // namespace custom_interfaces


namespace custom_interfaces
{

namespace action
{

namespace builder
{

class Init_Move_GetResult_Response_result
{
public:
  explicit Init_Move_GetResult_Response_result(::custom_interfaces::action::Move_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::custom_interfaces::action::Move_GetResult_Response result(::custom_interfaces::action::Move_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::action::Move_GetResult_Response msg_;
};

class Init_Move_GetResult_Response_status
{
public:
  Init_Move_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Move_GetResult_Response_result status(::custom_interfaces::action::Move_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_Move_GetResult_Response_result(msg_);
  }

private:
  ::custom_interfaces::action::Move_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::action::Move_GetResult_Response>()
{
  return custom_interfaces::action::builder::Init_Move_GetResult_Response_status();
}

}  // namespace custom_interfaces


namespace custom_interfaces
{

namespace action
{

namespace builder
{

class Init_Move_FeedbackMessage_feedback
{
public:
  explicit Init_Move_FeedbackMessage_feedback(::custom_interfaces::action::Move_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::custom_interfaces::action::Move_FeedbackMessage feedback(::custom_interfaces::action::Move_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::action::Move_FeedbackMessage msg_;
};

class Init_Move_FeedbackMessage_goal_id
{
public:
  Init_Move_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Move_FeedbackMessage_feedback goal_id(::custom_interfaces::action::Move_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_Move_FeedbackMessage_feedback(msg_);
  }

private:
  ::custom_interfaces::action::Move_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::action::Move_FeedbackMessage>()
{
  return custom_interfaces::action::builder::Init_Move_FeedbackMessage_goal_id();
}

}  // namespace custom_interfaces

#endif  // CUSTOM_INTERFACES__ACTION__DETAIL__MOVE__BUILDER_HPP_
