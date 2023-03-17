#include <linux/can/raw.h>
#include <linux/can.h>
#include <stdint.h>

#include "rclcpp/rclcpp.hpp"
#include "socketcan_interface.hpp"
#include "odrive_can.hpp"

#include "srv_and_msg/msg/odrive_status.hpp"

#include "srv_and_msg/srv/odrive_estop.hpp"
#include "srv_and_msg/srv/get_motor_error.hpp"
#include "srv_and_msg/srv/get_encoder_error.hpp"
#include "srv_and_msg/srv/set_axis_node_id.hpp"
#include "srv_and_msg/srv/set_axis_requested_state.hpp"
#include "srv_and_msg/srv/set_axis_startup_config.hpp"
#include "srv_and_msg/srv/get_encoder_estimates.hpp"
#include "srv_and_msg/srv/get_encoder_count.hpp"
#include "srv_and_msg/srv/set_controller_modes.hpp"
#include "srv_and_msg/srv/set_input_pos.hpp"
#include "srv_and_msg/srv/set_input_vel.hpp"
#include "srv_and_msg/srv/set_input_torque.hpp"
#include "srv_and_msg/srv/set_vel_limit.hpp"
#include "srv_and_msg/srv/start_anticogging.hpp"
#include "srv_and_msg/srv/set_traj_vel_limit.hpp"
#include "srv_and_msg/srv/set_traj_accel_limits.hpp"
#include "srv_and_msg/srv/set_traj_inertia.hpp"
#include "srv_and_msg/srv/get_iq.hpp"
#include "srv_and_msg/srv/reset_odrive.hpp"
#include "srv_and_msg/srv/get_vbus_voltage.hpp"
#include "srv_and_msg/srv/clear_errors.hpp"
#include "srv_and_msg/srv/get_temperature.hpp"
#include "srv_and_msg/srv/set_absolute_pos.hpp"
#include "srv_and_msg/srv/set_pos_gain.hpp"
#include "srv_and_msg/srv/set_vel_gains.hpp"



class CanService : public rclcpp::Node
{
public:
    CanService(/* args */);
    ~CanService();

private:
    rclcpp::Service<srv_and_msg::srv::OdriveEstop>::SharedPtr service_odrive_estop_;
    rclcpp::Service<srv_and_msg::srv::GetMotorError>::SharedPtr service_get_motor_error_;
    rclcpp::Service<srv_and_msg::srv::GetEncoderError>::SharedPtr service_get_encoder_error_;
    rclcpp::Service<srv_and_msg::srv::SetAxisNodeId>::SharedPtr service_set_axis_node_id_;
    rclcpp::Service<srv_and_msg::srv::SetAxisRequestedState>::SharedPtr service_set_axis_requested_state_;
    rclcpp::Service<srv_and_msg::srv::SetAxisStartupConfig>::SharedPtr service_set_axis_startup_config_;
    rclcpp::Service<srv_and_msg::srv::GetEncoderEstimates>::SharedPtr service_get_encoder_estimates_;
    rclcpp::Service<srv_and_msg::srv::GetEncoderCount>::SharedPtr service_get_encoder_count_;
    rclcpp::Service<srv_and_msg::srv::SetControllerModes>::SharedPtr service_set_controller_modes_;
    rclcpp::Service<srv_and_msg::srv::SetInputPos>::SharedPtr service_set_input_pos_;
    rclcpp::Service<srv_and_msg::srv::SetInputVel>::SharedPtr service_set_input_vel_;
    rclcpp::Service<srv_and_msg::srv::SetInputTorque>::SharedPtr service_set_input_torque_;
    rclcpp::Service<srv_and_msg::srv::SetVelLimit>::SharedPtr service_set_vel_limit_;
    rclcpp::Service<srv_and_msg::srv::StartAnticogging>::SharedPtr service_start_anticogging_;
    rclcpp::Service<srv_and_msg::srv::SetTrajVelLimit>::SharedPtr service_set_traj_vel_limit_;
    rclcpp::Service<srv_and_msg::srv::SetTrajAccelLimits>::SharedPtr service_set_traj_accel_limits_;
    rclcpp::Service<srv_and_msg::srv::SetTrajInertia>::SharedPtr service_set_traj_inertia_;
    rclcpp::Service<srv_and_msg::srv::GetIq>::SharedPtr service_get_iq_;
    rclcpp::Service<srv_and_msg::srv::GetTemperature>::SharedPtr service_get_temperature_;
    rclcpp::Service<srv_and_msg::srv::ResetOdrive>::SharedPtr service_reset_odrive_;
    rclcpp::Service<srv_and_msg::srv::GetVbusVoltage>::SharedPtr service_get_vbus_voltage_;
    rclcpp::Service<srv_and_msg::srv::ClearErrors>::SharedPtr service_clear_errors_;
    rclcpp::Service<srv_and_msg::srv::SetAbsolutePos>::SharedPtr service_set_absolute_pos_;
    rclcpp::Service<srv_and_msg::srv::SetPosGain>::SharedPtr service_set_pos_gain_;
    rclcpp::Service<srv_and_msg::srv::SetVelGains>::SharedPtr service_set_vel_gains_;



    SocketcanInterface socket_get_motor_error_ = SocketcanInterface(odrive_can::Msg::MSG_GET_MOTOR_ERROR);
    SocketcanInterface socket_get_encoder_error_ = SocketcanInterface(odrive_can::Msg::MSG_GET_ENCODER_ERROR);
    SocketcanInterface socket_get_encoder_estimates_ = SocketcanInterface(odrive_can::Msg::MSG_GET_ENCODER_ESTIMATES);
    SocketcanInterface socket_get_encoder_count_ = SocketcanInterface(odrive_can::Msg::MSG_GET_ENCODER_COUNT);
    SocketcanInterface socket_get_iq_ = SocketcanInterface(odrive_can::Msg::MSG_GET_IQ);
    SocketcanInterface socket_get_temperature_ = SocketcanInterface(odrive_can::Msg::MSG_GET_TEMPERATURE);
    SocketcanInterface socket_get_vbus_voltage_ = SocketcanInterface(odrive_can::Msg::MSG_GET_VBUS_VOLTAGE);
    SocketcanInterface socket_generic_write_ = SocketcanInterface(odrive_can::Msg::MSG_CO_NMT_CTRL);

    void odrive_estop_callback(const std::shared_ptr<srv_and_msg::srv::OdriveEstop::Request> request, std::shared_ptr<srv_and_msg::srv::OdriveEstop::Response> response);
    void get_motor_error_callback(const std::shared_ptr<srv_and_msg::srv::GetMotorError::Request> request, std::shared_ptr<srv_and_msg::srv::GetMotorError::Response> response);
    void get_encoder_error_callback(const std::shared_ptr<srv_and_msg::srv::GetEncoderError::Request> request, std::shared_ptr<srv_and_msg::srv::GetEncoderError::Response> response);
    void set_axis_node_id_callback(const std::shared_ptr<srv_and_msg::srv::SetAxisNodeId::Request> request, std::shared_ptr<srv_and_msg::srv::SetAxisNodeId::Response> response);
    void set_axis_requested_state_callback(const std::shared_ptr<srv_and_msg::srv::SetAxisRequestedState::Request> request, std::shared_ptr<srv_and_msg::srv::SetAxisRequestedState::Response> response);
    void set_axis_startup_config_callback(const std::shared_ptr<srv_and_msg::srv::SetAxisStartupConfig::Request> request, std::shared_ptr<srv_and_msg::srv::SetAxisStartupConfig::Response> response);
    void get_encoder_estimates_callback(const std::shared_ptr<srv_and_msg::srv::GetEncoderEstimates::Request> request, std::shared_ptr<srv_and_msg::srv::GetEncoderEstimates::Response> response);
    void get_encoder_count_callback(const std::shared_ptr<srv_and_msg::srv::GetEncoderCount::Request> request, std::shared_ptr<srv_and_msg::srv::GetEncoderCount::Response> response);
    void set_controller_modes_callback(const std::shared_ptr<srv_and_msg::srv::SetControllerModes::Request> request, std::shared_ptr<srv_and_msg::srv::SetControllerModes::Response> response);
    void set_input_pos_callback(const std::shared_ptr<srv_and_msg::srv::SetInputPos::Request> request, std::shared_ptr<srv_and_msg::srv::SetInputPos::Response> response);
    void set_input_vel_callback(const std::shared_ptr<srv_and_msg::srv::SetInputVel::Request> request, std::shared_ptr<srv_and_msg::srv::SetInputVel::Response> response);
    void set_input_torque_callback(const std::shared_ptr<srv_and_msg::srv::SetInputTorque::Request> request, std::shared_ptr<srv_and_msg::srv::SetInputTorque::Response> response);
    void set_vel_limit_callback(const std::shared_ptr<srv_and_msg::srv::SetVelLimit::Request> request, std::shared_ptr<srv_and_msg::srv::SetVelLimit::Response> response);
    void start_anticogging_callback(const std::shared_ptr<srv_and_msg::srv::StartAnticogging::Request> request, std::shared_ptr<srv_and_msg::srv::StartAnticogging::Response> response);
    void set_traj_vel_limit_callback(const std::shared_ptr<srv_and_msg::srv::SetTrajVelLimit::Request> request, std::shared_ptr<srv_and_msg::srv::SetTrajVelLimit::Response> response);
    void set_traj_accel_limits_callback(const std::shared_ptr<srv_and_msg::srv::SetTrajAccelLimits::Request> request, std::shared_ptr<srv_and_msg::srv::SetTrajAccelLimits::Response> response);
    void set_traj_inertia_callback(const std::shared_ptr<srv_and_msg::srv::SetTrajInertia::Request> request, std::shared_ptr<srv_and_msg::srv::SetTrajInertia::Response> response);
    void get_iq_callback(const std::shared_ptr<srv_and_msg::srv::GetIq::Request> request, std::shared_ptr<srv_and_msg::srv::GetIq::Response> response);
    void get_temperature_callback(const std::shared_ptr<srv_and_msg::srv::GetTemperature::Request> request, std::shared_ptr<srv_and_msg::srv::GetTemperature::Response> response);
    void reset_odrive_callback(const std::shared_ptr<srv_and_msg::srv::ResetOdrive::Request> request, std::shared_ptr<srv_and_msg::srv::ResetOdrive::Response> response);
    void get_vbus_voltage_callback(const std::shared_ptr<srv_and_msg::srv::GetVbusVoltage::Request> request, std::shared_ptr<srv_and_msg::srv::GetVbusVoltage::Response> response);
    void clear_errors_callback(const std::shared_ptr<srv_and_msg::srv::ClearErrors::Request> request, std::shared_ptr<srv_and_msg::srv::ClearErrors::Response> response);
    void set_absolute_pos_callback(const std::shared_ptr<srv_and_msg::srv::SetAbsolutePos::Request> request, std::shared_ptr<srv_and_msg::srv::SetAbsolutePos::Response> response);
    void set_pos_gain_callback(const std::shared_ptr<srv_and_msg::srv::SetPosGain::Request> request, std::shared_ptr<srv_and_msg::srv::SetPosGain::Response> response);
    void set_vel_gains_callback(const std::shared_ptr<srv_and_msg::srv::SetVelGains::Request> request, std::shared_ptr<srv_and_msg::srv::SetVelGains::Response> response);
};