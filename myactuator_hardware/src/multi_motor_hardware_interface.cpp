#include "myactuator_hardware/multi_motor_hardware_interface.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <fstream>
#include <cerrno>
#include <cstring>

#include <hardware_interface/types/hardware_interface_type_values.hpp>

// For CAN interface checking
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <unistd.h>

namespace myactuator_hardware
{

// ============================================================================
// Helper: Print formatted error box
// ============================================================================
void printErrorBox(rclcpp::Logger logger, const std::string& title, 
                   const std::vector<std::string>& lines)
{
  RCLCPP_ERROR(logger, " ");
  RCLCPP_ERROR(logger, "╔══════════════════════════════════════════════════════════════╗");
  RCLCPP_ERROR(logger, "║  %-60s║", title.c_str());
  RCLCPP_ERROR(logger, "╠══════════════════════════════════════════════════════════════╣");
  for (const auto& line : lines) {
    RCLCPP_ERROR(logger, "║  %-60s║", line.c_str());
  }
  RCLCPP_ERROR(logger, "╚══════════════════════════════════════════════════════════════╝");
  RCLCPP_ERROR(logger, " ");
}

void printInfoBox(rclcpp::Logger logger, const std::string& title,
                  const std::vector<std::string>& lines)
{
  RCLCPP_INFO(logger, " ");
  RCLCPP_INFO(logger, "┌──────────────────────────────────────────────────────────────┐");
  RCLCPP_INFO(logger, "│  %-60s│", title.c_str());
  RCLCPP_INFO(logger, "├──────────────────────────────────────────────────────────────┤");
  for (const auto& line : lines) {
    RCLCPP_INFO(logger, "│  %-60s│", line.c_str());
  }
  RCLCPP_INFO(logger, "└──────────────────────────────────────────────────────────────┘");
  RCLCPP_INFO(logger, " ");
}

// ============================================================================
// Destructor
// ============================================================================
MultiMotorHardwareInterface::~MultiMotorHardwareInterface()
{
  on_cleanup(rclcpp_lifecycle::State());
}

rclcpp::Logger MultiMotorHardwareInterface::getLogger() const
{
  return rclcpp::get_logger("MyActuator_HW");
}

// ============================================================================
// on_init: Parse URDF parameters
// ============================================================================
hardware_interface::CallbackReturn MultiMotorHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Parse CAN interface name
  if (info_.hardware_parameters.find("ifname") == info_.hardware_parameters.end()) {
    printErrorBox(getLogger(), "CONFIGURATION ERROR",
      {"Missing required parameter: 'ifname'",
       "",
       "Add to your ros2_control URDF:",
       "  <param name=\"ifname\">can0</param>"});
    return hardware_interface::CallbackReturn::ERROR;
  }
  ifname_ = info_.hardware_parameters.at("ifname");

  // Parse optional parameters
  if (info_.hardware_parameters.find("cycle_time") != info_.hardware_parameters.end()) {
    cycle_time_ = std::chrono::milliseconds(
      std::stol(info_.hardware_parameters.at("cycle_time")));
  }
  if (info_.hardware_parameters.find("timeout") != info_.hardware_parameters.end()) {
    timeout_ = std::chrono::milliseconds(
      std::stoi(info_.hardware_parameters.at("timeout")));
  }

  // Initialize motor data
  motors_.resize(info_.joints.size());
  
  RCLCPP_INFO(getLogger(), "Initializing %zu joints on CAN interface '%s'",
    info_.joints.size(), ifname_.c_str());

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    if (!initializeMotor(info_.joints[i], i)) {
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(getLogger(), "✓ Configuration parsed successfully");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// initializeMotor: Parse joint parameters
// ============================================================================
bool MultiMotorHardwareInterface::initializeMotor(
  const hardware_interface::ComponentInfo & joint, size_t index)
{
  MotorData & motor = motors_[index];
  motor.joint_name = joint.name;

  // Parse actuator_id (required)
  if (joint.parameters.find("actuator_id") == joint.parameters.end()) {
    printErrorBox(getLogger(), "CONFIGURATION ERROR",
      {"Joint '" + joint.name + "' missing 'actuator_id'",
       "",
       "Add to your URDF joint definition:",
       "  <param name=\"actuator_id\">1</param>"});
    return false;
  }
  motor.actuator_id = std::stoi(joint.parameters.at("actuator_id"));

  // Parse torque_constant (optional)
  if (joint.parameters.find("torque_constant") != joint.parameters.end()) {
    motor.torque_constant = std::stod(joint.parameters.at("torque_constant"));
  } else {
    motor.torque_constant = 1.0;
  }

  RCLCPP_INFO(getLogger(), "  → Joint[%zu]: '%s' → Motor ID=%d, Kt=%.3f",
    index, motor.joint_name.c_str(), motor.actuator_id, motor.torque_constant);

  return true;
}

// ============================================================================
// checkCANInterface: Verify CAN interface status
// ============================================================================
bool MultiMotorHardwareInterface::checkCANInterface()
{
  std::vector<std::string> available_can;
  
  // Check /sys/class/net for CAN interfaces
  std::ifstream proc_net("/proc/net/dev");
  if (proc_net.is_open()) {
    std::string line;
    while (std::getline(proc_net, line)) {
      if (line.find("can") != std::string::npos || line.find("vcan") != std::string::npos) {
        size_t pos = line.find(':');
        if (pos != std::string::npos) {
          std::string iface = line.substr(0, pos);
          // Trim whitespace
          iface.erase(0, iface.find_first_not_of(" \t"));
          iface.erase(iface.find_last_not_of(" \t") + 1);
          available_can.push_back(iface);
        }
      }
    }
  }

  // Try to create socket and check interface
  int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (sock < 0) {
    printErrorBox(getLogger(), "CAN SOCKET ERROR",
      {"Failed to create CAN socket: " + std::string(std::strerror(errno)),
       "",
       "Try running:",
       "  sudo modprobe can",
       "  sudo modprobe can_raw"});
    return false;
  }

  struct ifreq ifr;
  std::strncpy(ifr.ifr_name, ifname_.c_str(), IFNAMSIZ - 1);
  ifr.ifr_name[IFNAMSIZ - 1] = '\0';

  if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0) {
    close(sock);
    
    std::vector<std::string> error_lines = {
      "Interface '" + ifname_ + "' not found!",
      ""
    };
    
    if (!available_can.empty()) {
      error_lines.push_back("Available CAN interfaces:");
      for (const auto& iface : available_can) {
        error_lines.push_back("  • " + iface);
      }
      error_lines.push_back("");
    }
    
    error_lines.push_back("To set up CAN interface:");
    error_lines.push_back("  # For real hardware:");
    error_lines.push_back("  sudo ip link set " + ifname_ + " type can bitrate 1000000");
    error_lines.push_back("  sudo ip link set " + ifname_ + " up");
    error_lines.push_back("");
    error_lines.push_back("  # For virtual CAN (testing):");
    error_lines.push_back("  sudo modprobe vcan");
    error_lines.push_back("  sudo ip link add dev " + ifname_ + " type vcan");
    error_lines.push_back("  sudo ip link set " + ifname_ + " up");
    
    printErrorBox(getLogger(), "CAN INTERFACE NOT FOUND", error_lines);
    return false;
  }

  // Check if interface is UP
  if (ioctl(sock, SIOCGIFFLAGS, &ifr) >= 0) {
    if (!(ifr.ifr_flags & IFF_UP)) {
      close(sock);
      printErrorBox(getLogger(), "CAN INTERFACE DOWN",
        {"Interface '" + ifname_ + "' exists but is DOWN",
         "",
         "Run: sudo ip link set " + ifname_ + " up"});
      return false;
    }
  }

  close(sock);
  RCLCPP_INFO(getLogger(), "✓ CAN interface '%s' is UP", ifname_.c_str());
  return true;
}

// ============================================================================
// on_configure: Connect to motors
// ============================================================================
hardware_interface::CallbackReturn MultiMotorHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(getLogger(), " ");
  RCLCPP_INFO(getLogger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  RCLCPP_INFO(getLogger(), "  Configuring MyActuator Hardware Interface");
  RCLCPP_INFO(getLogger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");

  // Step 1: Check CAN interface
  RCLCPP_INFO(getLogger(), "[1/3] Checking CAN interface '%s'...", ifname_.c_str());
  if (!checkCANInterface()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Step 2: Create CAN driver
  RCLCPP_INFO(getLogger(), "[2/3] Creating CAN driver...");
  try {
    driver_ = std::make_unique<myactuator_rmd::CanDriver>(ifname_);
    RCLCPP_INFO(getLogger(), "✓ CAN driver created");
  } catch (const std::exception & e) {
    printErrorBox(getLogger(), "CAN DRIVER ERROR",
      {"Failed to create CAN driver: " + std::string(e.what()),
       "",
       "Check that no other process is using the interface"});
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Step 3: Connect to each motor
  RCLCPP_INFO(getLogger(), "[3/3] Connecting to motors...");
  
  for (auto & motor : motors_) {
    RCLCPP_INFO(getLogger(), "  → Connecting to '%s' (ID=%d)...", 
      motor.joint_name.c_str(), motor.actuator_id);
    
    try {
      motor.actuator = std::make_unique<myactuator_rmd::ActuatorInterface>(
        *driver_, motor.actuator_id);
      
      // Verify connection by querying motor model
      std::string model = motor.actuator->getMotorModel();
      RCLCPP_INFO(getLogger(), "    ✓ Connected: %s", model.c_str());
      
    } catch (const std::exception & e) {
      std::vector<std::string> error_lines = {
        "Motor: " + motor.joint_name,
        "ID: " + std::to_string(motor.actuator_id),
        "Error: " + std::string(e.what()),
        "",
        "Checklist:",
        "  □ Motor power is ON",
        "  □ CAN wiring correct (CAN_H, CAN_L, GND)",
        "  □ Motor ID matches configuration",
        "  □ CAN bitrate matches motor (usually 1Mbps)",
        "  □ 120Ω termination resistor installed"
      };
      printErrorBox(getLogger(), "MOTOR CONNECTION FAILED", error_lines);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // Start async thread
  if (!startAsyncThread()) {
    RCLCPP_ERROR(getLogger(), "Failed to start async thread");
    return hardware_interface::CallbackReturn::ERROR;
  }

  printInfoBox(getLogger(), "CONFIGURATION SUCCESSFUL",
    {"All " + std::to_string(motors_.size()) + " motors connected and ready",
     "CAN Interface: " + ifname_});

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// on_cleanup
// ============================================================================
hardware_interface::CallbackReturn MultiMotorHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(getLogger(), "Cleaning up...");

  stopAsyncThread();

  for (auto & motor : motors_) {
    if (motor.actuator) {
      try {
        motor.actuator->shutdownMotor();
      } catch (...) {}
      motor.actuator.reset();
    }
  }
  driver_.reset();

  RCLCPP_INFO(getLogger(), "✓ Cleanup complete");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// on_activate
// ============================================================================
hardware_interface::CallbackReturn MultiMotorHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(getLogger(), "Activating motors...");

  for (auto & motor : motors_) {
    motor.position_command = motor.position_state;
    motor.velocity_command = 0.0;
    motor.effort_command = 0.0;
    RCLCPP_INFO(getLogger(), "  → '%s': ready at %.3f rad", 
      motor.joint_name.c_str(), motor.position_state);
  }

  RCLCPP_INFO(getLogger(), "✓ All motors activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// on_deactivate
// ============================================================================
hardware_interface::CallbackReturn MultiMotorHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(getLogger(), "Deactivating motors...");

  for (auto & motor : motors_) {
    if (motor.actuator) {
      try {
        motor.actuator->stopMotor();
        RCLCPP_INFO(getLogger(), "  → '%s': stopped", motor.joint_name.c_str());
      } catch (const std::exception & e) {
        RCLCPP_WARN(getLogger(), "  → '%s': stop failed (%s)", 
          motor.joint_name.c_str(), e.what());
      }
    }
  }

  RCLCPP_INFO(getLogger(), "✓ Deactivation complete");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// on_error
// ============================================================================
hardware_interface::CallbackReturn MultiMotorHardwareInterface::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  printErrorBox(getLogger(), "HARDWARE ERROR",
    {"Emergency stop triggered",
     "All motors will be stopped"});

  stopAsyncThread();

  for (auto & motor : motors_) {
    if (motor.actuator) {
      try {
        motor.actuator->stopMotor();
        motor.actuator->reset();
      } catch (...) {}
    }
  }

  RCLCPP_INFO(getLogger(), "✓ Hardware in safe state");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// export_state_interfaces
// ============================================================================
std::vector<hardware_interface::StateInterface>
MultiMotorHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (auto & motor : motors_) {
    state_interfaces.emplace_back(
      motor.joint_name, hardware_interface::HW_IF_POSITION, &motor.position_state);
    state_interfaces.emplace_back(
      motor.joint_name, hardware_interface::HW_IF_VELOCITY, &motor.velocity_state);
    state_interfaces.emplace_back(
      motor.joint_name, hardware_interface::HW_IF_EFFORT, &motor.effort_state);
  }

  RCLCPP_DEBUG(getLogger(), "Exported %zu state interfaces", state_interfaces.size());
  return state_interfaces;
}

// ============================================================================
// export_command_interfaces
// ============================================================================
std::vector<hardware_interface::CommandInterface>
MultiMotorHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (auto & motor : motors_) {
    command_interfaces.emplace_back(
      motor.joint_name, hardware_interface::HW_IF_POSITION, &motor.position_command);
    command_interfaces.emplace_back(
      motor.joint_name, hardware_interface::HW_IF_VELOCITY, &motor.velocity_command);
    command_interfaces.emplace_back(
      motor.joint_name, hardware_interface::HW_IF_EFFORT, &motor.effort_command);
  }

  RCLCPP_DEBUG(getLogger(), "Exported %zu command interfaces", command_interfaces.size());
  return command_interfaces;
}

// ============================================================================
// read: Get motor feedback
// ============================================================================
hardware_interface::return_type MultiMotorHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::lock_guard<std::mutex> lock(mutex_);

  for (auto & motor : motors_) {
    if (!motor.actuator) continue;

    try {
      auto feedback = motor.actuator->getMotorStatus2();
      
      motor.position_state = feedback.shaft_angle * M_PI / 180.0;
      motor.velocity_state = feedback.shaft_speed * M_PI / 180.0;
      motor.effort_state = feedback.current * motor.torque_constant;

    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(getLogger(), *rclcpp::Clock::make_shared(), 5000,
        "Read failed for '%s': %s", motor.joint_name.c_str(), e.what());
    }
  }

  return hardware_interface::return_type::OK;
}

// ============================================================================
// write: Send motor commands
// ============================================================================
hardware_interface::return_type MultiMotorHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::lock_guard<std::mutex> lock(mutex_);

  for (auto & motor : motors_) {
    if (!motor.actuator) continue;

    try {
      double position_deg = motor.position_command * 180.0 / M_PI;
      double velocity_dps = std::abs(motor.velocity_command) * 180.0 / M_PI;
      
      if (velocity_dps < 1.0) {
        velocity_dps = 360.0;  // Default max velocity
      }

      motor.actuator->sendPositionAbsoluteSetpoint(position_deg, velocity_dps);

    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(getLogger(), *rclcpp::Clock::make_shared(), 5000,
        "Write failed for '%s': %s", motor.joint_name.c_str(), e.what());
    }
  }

  return hardware_interface::return_type::OK;
}

// ============================================================================
// Async thread management
// ============================================================================
bool MultiMotorHardwareInterface::startAsyncThread()
{
  stop_async_thread_.store(false);
  async_thread_ = std::thread(&MultiMotorHardwareInterface::asyncThreadFunc, this);
  return true;
}

void MultiMotorHardwareInterface::stopAsyncThread()
{
  stop_async_thread_.store(true);
  if (async_thread_.joinable()) {
    async_thread_.join();
  }
}

void MultiMotorHardwareInterface::asyncThreadFunc()
{
  while (!stop_async_thread_.load()) {
    std::this_thread::sleep_for(cycle_time_);
  }
}

}  // namespace myactuator_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  myactuator_hardware::MultiMotorHardwareInterface,
  hardware_interface::SystemInterface)
