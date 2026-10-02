// Prints the sizes, field offsets, command/status enum values, and protocol version of libfranka's
// robot wire-format types, for comparison against the franka-rs mirror structs in
// source/wire/robot.rs.
//
// Needs only the C++ standard library and libfranka's common headers (no Eigen, no Pinocchio, no
// built library), so it compiles anywhere with a C++17 compiler, e.g.:
//
//   c++ -std=c++17 -I external/libfranka/common/include \
//       validation/reference/message_layout.cpp -o message_layout
//   ./message_layout > validation/data/message_layout.txt
//
// Output is one "key value" record per line. Keys name the franka-rs item being checked:
//   version                        the protocol version (ConnectRequest.version / kVersion)
//   command.<Variant>              the discriminant of each Command enum variant
//   status.<Enum>.<Variant>        the discriminant of each status enum variant
//   size.<RustStruct>              sizeof the libfranka struct that the named franka-rs struct mirrors
//   offset.<RustStruct>.<field>    byte offset of a field, keyed by its franka-rs field name
//
// The companion tests in source/wire/robot.rs read this file and assert the franka-rs values match.

#include <cstddef>
#include <cstdint>
#include <iostream>

#include <research_interface/robot/rbk_types.h>
#include <research_interface/robot/service_types.h>

using namespace research_interface::robot;

// Prints one "key value" record.
template <typename T>
static void record(const char* key, T value) {
  std::cout << key << ' ' << static_cast<std::uint64_t>(value) << '\n';
}

// Emits the byte offset of a libfranka field, keyed by the franka-rs struct and field names.
#define OFFSET(rust_struct, rust_field, cpp_type, cpp_field) \
  std::cout << "offset." #rust_struct "." #rust_field " " << offsetof(cpp_type, cpp_field) << '\n'

// Emits the offset of a franka-rs field that franka-rs flattens out of a Move::Deviation member.
#define DEVIATION_OFFSET(rust_field, cpp_member, cpp_dev_field)                    \
  std::cout << "offset.MoveRequest." #rust_field " "                              \
            << (offsetof(Move::Request, cpp_member) + offsetof(Move::Deviation, cpp_dev_field)) \
            << '\n'

// Emits the discriminant of a status enum variant, keyed by the franka-rs enum and variant names.
#define STATUS(rust_enum, rust_variant, cpp_value) \
  record("status." #rust_enum "." #rust_variant, cpp_value)

int main() {
  record("version", kVersion);

  record("command.Connect", Command::kConnect);
  record("command.Move", Command::kMove);
  record("command.StopMove", Command::kStopMove);
  record("command.SetCollisionBehavior", Command::kSetCollisionBehavior);
  record("command.SetJointImpedance", Command::kSetJointImpedance);
  record("command.SetCartesianImpedance", Command::kSetCartesianImpedance);
  record("command.SetGuidingMode", Command::kSetGuidingMode);
  record("command.SetEeToK", Command::kSetEEToK);
  record("command.SetNeToEe", Command::kSetNEToEE);
  record("command.SetLoad", Command::kSetLoad);
  record("command.AutomaticErrorRecovery", Command::kAutomaticErrorRecovery);
  record("command.GetRobotModel", Command::kGetRobotModel);

  // Status enums. The getter/setter status is shared by all GetterSetterCommandBase commands;
  // SetJointImpedance is one such command, so its Status is that shared enum.
  STATUS(ConnectStatus, Success, Connect::Status::kSuccess);
  STATUS(ConnectStatus, IncompatibleLibraryVersion, Connect::Status::kIncompatibleLibraryVersion);

  STATUS(MoveStatus, Success, Move::Status::kSuccess);
  STATUS(MoveStatus, MotionStarted, Move::Status::kMotionStarted);
  STATUS(MoveStatus, Preempted, Move::Status::kPreempted);
  STATUS(MoveStatus, PreemptedDueToActivatedSafetyFunctions,
         Move::Status::kPreemptedDueToActivatedSafetyFunctions);
  STATUS(MoveStatus, CommandRejectedDueToActivatedSafetyFunctions,
         Move::Status::kCommandRejectedDueToActivatedSafetyFunctions);
  STATUS(MoveStatus, CommandNotPossibleRejected, Move::Status::kCommandNotPossibleRejected);
  STATUS(MoveStatus, StartAtSingularPoseRejected, Move::Status::kStartAtSingularPoseRejected);
  STATUS(MoveStatus, InvalidArgumentRejected, Move::Status::kInvalidArgumentRejected);
  STATUS(MoveStatus, ReflexAborted, Move::Status::kReflexAborted);
  STATUS(MoveStatus, EmergencyAborted, Move::Status::kEmergencyAborted);
  STATUS(MoveStatus, InputErrorAborted, Move::Status::kInputErrorAborted);
  STATUS(MoveStatus, Aborted, Move::Status::kAborted);

  STATUS(GetterSetterStatus, Success, SetJointImpedance::Status::kSuccess);
  STATUS(GetterSetterStatus, CommandNotPossibleRejected,
         SetJointImpedance::Status::kCommandNotPossibleRejected);
  STATUS(GetterSetterStatus, InvalidArgumentRejected,
         SetJointImpedance::Status::kInvalidArgumentRejected);
  STATUS(GetterSetterStatus, CommandRejectedDueToActivatedSafetyFunctions,
         SetJointImpedance::Status::kCommandRejectedDueToActivatedSafetyFunctions);

  STATUS(StopMoveStatus, Success, StopMove::Status::kSuccess);
  STATUS(StopMoveStatus, CommandNotPossibleRejected, StopMove::Status::kCommandNotPossibleRejected);
  STATUS(StopMoveStatus, CommandRejectedDueToActivatedSafetyFunctions,
         StopMove::Status::kCommandRejectedDueToActivatedSafetyFunctions);
  STATUS(StopMoveStatus, EmergencyAborted, StopMove::Status::kEmergencyAborted);
  STATUS(StopMoveStatus, ReflexAborted, StopMove::Status::kReflexAborted);
  STATUS(StopMoveStatus, Aborted, StopMove::Status::kAborted);

  STATUS(AutomaticErrorRecoveryStatus, Success, AutomaticErrorRecovery::Status::kSuccess);
  STATUS(AutomaticErrorRecoveryStatus, CommandNotPossibleRejected,
         AutomaticErrorRecovery::Status::kCommandNotPossibleRejected);
  STATUS(AutomaticErrorRecoveryStatus, CommandRejectedDueToActivatedSafetyFunctions,
         AutomaticErrorRecovery::Status::kCommandRejectedDueToActivatedSafetyFunctions);
  STATUS(AutomaticErrorRecoveryStatus, ManualErrorRecoveryRequiredRejected,
         AutomaticErrorRecovery::Status::kManualErrorRecoveryRequiredRejected);
  STATUS(AutomaticErrorRecoveryStatus, ReflexAborted, AutomaticErrorRecovery::Status::kReflexAborted);
  STATUS(AutomaticErrorRecoveryStatus, EmergencyAborted,
         AutomaticErrorRecovery::Status::kEmergencyAborted);
  STATUS(AutomaticErrorRecoveryStatus, Aborted, AutomaticErrorRecovery::Status::kAborted);

  // TCP command channel: the fixed-size request/response payloads.
  record("size.CommandHeader", sizeof(CommandHeader));
  record("size.ConnectRequest", sizeof(Connect::Request));
  record("size.ConnectResponse", sizeof(Connect::Response));
  record("size.MoveRequest", sizeof(Move::Request));
  record("size.SetCollisionBehaviorRequest", sizeof(SetCollisionBehavior::Request));
  record("size.SetJointImpedanceRequest", sizeof(SetJointImpedance::Request));
  record("size.SetCartesianImpedanceRequest", sizeof(SetCartesianImpedance::Request));
  record("size.SetGuidingModeRequest", sizeof(SetGuidingMode::Request));
  record("size.SetEeToKRequest", sizeof(SetEEToK::Request));
  record("size.SetNeToEeRequest", sizeof(SetNEToEE::Request));
  record("size.SetLoadRequest", sizeof(SetLoad::Request));

  // UDP realtime channel: the robot state and the command sent each control cycle.
  record("size.RawRobotState", sizeof(RobotState));
  record("size.MotionGeneratorCommand", sizeof(MotionGeneratorCommand));
  record("size.ControllerCommand", sizeof(ControllerCommand));
  record("size.RobotCommand", sizeof(RobotCommand));

  // Field offsets. ConnectResponse is omitted: it is not standard-layout in C++ (its status lives
  // in a base class), so offsetof is not well-defined on it; its 3-byte size check pins its layout.
  OFFSET(CommandHeader, command, CommandHeader, command);
  OFFSET(CommandHeader, command_id, CommandHeader, command_id);
  OFFSET(CommandHeader, size, CommandHeader, size);

  OFFSET(ConnectRequest, version, Connect::Request, version);
  OFFSET(ConnectRequest, udp_port, Connect::Request, udp_port);

  OFFSET(MoveRequest, controller_mode, Move::Request, controller_mode);
  OFFSET(MoveRequest, motion_generator_mode, Move::Request, motion_generator_mode);
  DEVIATION_OFFSET(maximum_path_deviation_translation, maximum_path_deviation, translation);
  DEVIATION_OFFSET(maximum_path_deviation_rotation, maximum_path_deviation, rotation);
  DEVIATION_OFFSET(maximum_path_deviation_elbow, maximum_path_deviation, elbow);
  DEVIATION_OFFSET(maximum_goal_pose_deviation_translation, maximum_goal_pose_deviation, translation);
  DEVIATION_OFFSET(maximum_goal_pose_deviation_rotation, maximum_goal_pose_deviation, rotation);
  DEVIATION_OFFSET(maximum_goal_pose_deviation_elbow, maximum_goal_pose_deviation, elbow);
  OFFSET(MoveRequest, use_async_motion_generator, Move::Request, use_async_motion_generator);
  OFFSET(MoveRequest, maximum_velocity, Move::Request, maximum_velocity);

  OFFSET(SetCollisionBehaviorRequest, lower_torque_thresholds_acceleration,
         SetCollisionBehavior::Request, lower_torque_thresholds_acceleration);
  OFFSET(SetCollisionBehaviorRequest, upper_torque_thresholds_acceleration,
         SetCollisionBehavior::Request, upper_torque_thresholds_acceleration);
  OFFSET(SetCollisionBehaviorRequest, lower_torque_thresholds_nominal, SetCollisionBehavior::Request,
         lower_torque_thresholds_nominal);
  OFFSET(SetCollisionBehaviorRequest, upper_torque_thresholds_nominal, SetCollisionBehavior::Request,
         upper_torque_thresholds_nominal);
  OFFSET(SetCollisionBehaviorRequest, lower_force_thresholds_acceleration,
         SetCollisionBehavior::Request, lower_force_thresholds_acceleration);
  OFFSET(SetCollisionBehaviorRequest, upper_force_thresholds_acceleration,
         SetCollisionBehavior::Request, upper_force_thresholds_acceleration);
  OFFSET(SetCollisionBehaviorRequest, lower_force_thresholds_nominal, SetCollisionBehavior::Request,
         lower_force_thresholds_nominal);
  OFFSET(SetCollisionBehaviorRequest, upper_force_thresholds_nominal, SetCollisionBehavior::Request,
         upper_force_thresholds_nominal);

  OFFSET(SetJointImpedanceRequest, k_theta, SetJointImpedance::Request, K_theta);
  OFFSET(SetCartesianImpedanceRequest, k_x, SetCartesianImpedance::Request, K_x);
  OFFSET(SetGuidingModeRequest, guiding_mode, SetGuidingMode::Request, guiding_mode);
  OFFSET(SetGuidingModeRequest, nullspace, SetGuidingMode::Request, nullspace);
  OFFSET(SetEeToKRequest, ee_t_k, SetEEToK::Request, EE_T_K);
  OFFSET(SetNeToEeRequest, ne_t_ee, SetNEToEE::Request, NE_T_EE);
  OFFSET(SetLoadRequest, m_load, SetLoad::Request, m_load);
  OFFSET(SetLoadRequest, f_x_cload, SetLoad::Request, F_x_Cload);
  OFFSET(SetLoadRequest, i_load, SetLoad::Request, I_load);

  OFFSET(RawRobotState, message_id, RobotState, message_id);
  OFFSET(RawRobotState, o_t_ee, RobotState, O_T_EE);
  OFFSET(RawRobotState, o_t_ee_d, RobotState, O_T_EE_d);
  OFFSET(RawRobotState, f_t_ee, RobotState, F_T_EE);
  OFFSET(RawRobotState, ee_t_k, RobotState, EE_T_K);
  OFFSET(RawRobotState, f_t_ne, RobotState, F_T_NE);
  OFFSET(RawRobotState, ne_t_ee, RobotState, NE_T_EE);
  OFFSET(RawRobotState, m_ee, RobotState, m_ee);
  OFFSET(RawRobotState, i_ee, RobotState, I_ee);
  OFFSET(RawRobotState, f_x_cee, RobotState, F_x_Cee);
  OFFSET(RawRobotState, m_load, RobotState, m_load);
  OFFSET(RawRobotState, i_load, RobotState, I_load);
  OFFSET(RawRobotState, f_x_cload, RobotState, F_x_Cload);
  OFFSET(RawRobotState, elbow, RobotState, elbow);
  OFFSET(RawRobotState, elbow_d, RobotState, elbow_d);
  OFFSET(RawRobotState, tau_j, RobotState, tau_J);
  OFFSET(RawRobotState, tau_j_d, RobotState, tau_J_d);
  OFFSET(RawRobotState, dtau_j, RobotState, dtau_J);
  OFFSET(RawRobotState, q, RobotState, q);
  OFFSET(RawRobotState, q_d, RobotState, q_d);
  OFFSET(RawRobotState, dq, RobotState, dq);
  OFFSET(RawRobotState, dq_d, RobotState, dq_d);
  OFFSET(RawRobotState, ddq_d, RobotState, ddq_d);
  OFFSET(RawRobotState, joint_contact, RobotState, joint_contact);
  OFFSET(RawRobotState, cartesian_contact, RobotState, cartesian_contact);
  OFFSET(RawRobotState, joint_collision, RobotState, joint_collision);
  OFFSET(RawRobotState, cartesian_collision, RobotState, cartesian_collision);
  OFFSET(RawRobotState, tau_ext_hat_filtered, RobotState, tau_ext_hat_filtered);
  OFFSET(RawRobotState, o_f_ext_hat_k, RobotState, O_F_ext_hat_K);
  OFFSET(RawRobotState, k_f_ext_hat_k, RobotState, K_F_ext_hat_K);
  OFFSET(RawRobotState, o_dp_ee_d, RobotState, O_dP_EE_d);
  OFFSET(RawRobotState, o_ddp_o, RobotState, O_ddP_O);
  OFFSET(RawRobotState, elbow_c, RobotState, elbow_c);
  OFFSET(RawRobotState, delbow_c, RobotState, delbow_c);
  OFFSET(RawRobotState, ddelbow_c, RobotState, ddelbow_c);
  OFFSET(RawRobotState, o_t_ee_c, RobotState, O_T_EE_c);
  OFFSET(RawRobotState, o_dp_ee_c, RobotState, O_dP_EE_c);
  OFFSET(RawRobotState, o_ddp_ee_c, RobotState, O_ddP_EE_c);
  OFFSET(RawRobotState, theta, RobotState, theta);
  OFFSET(RawRobotState, dtheta, RobotState, dtheta);
  OFFSET(RawRobotState, accelerometer_top, RobotState, accelerometer_top);
  OFFSET(RawRobotState, accelerometer_bottom, RobotState, accelerometer_bottom);
  OFFSET(RawRobotState, motion_generator_mode, RobotState, motion_generator_mode);
  OFFSET(RawRobotState, controller_mode, RobotState, controller_mode);
  OFFSET(RawRobotState, errors, RobotState, errors);
  OFFSET(RawRobotState, reflex_reason, RobotState, reflex_reason);
  OFFSET(RawRobotState, robot_mode, RobotState, robot_mode);
  OFFSET(RawRobotState, control_command_success_rate, RobotState, control_command_success_rate);

  OFFSET(MotionGeneratorCommand, q_c, MotionGeneratorCommand, q_c);
  OFFSET(MotionGeneratorCommand, dq_c, MotionGeneratorCommand, dq_c);
  OFFSET(MotionGeneratorCommand, o_t_ee_c, MotionGeneratorCommand, O_T_EE_c);
  OFFSET(MotionGeneratorCommand, o_dp_ee_c, MotionGeneratorCommand, O_dP_EE_c);
  OFFSET(MotionGeneratorCommand, elbow_c, MotionGeneratorCommand, elbow_c);
  OFFSET(MotionGeneratorCommand, valid_elbow, MotionGeneratorCommand, valid_elbow);
  OFFSET(MotionGeneratorCommand, motion_generation_finished, MotionGeneratorCommand,
         motion_generation_finished);

  OFFSET(ControllerCommand, tau_j_d, ControllerCommand, tau_J_d);
  OFFSET(ControllerCommand, torque_command_finished, ControllerCommand, torque_command_finished);

  OFFSET(RobotCommand, message_id, RobotCommand, message_id);
  OFFSET(RobotCommand, motion, RobotCommand, motion);
  OFFSET(RobotCommand, control, RobotCommand, control);

  return 0;
}
