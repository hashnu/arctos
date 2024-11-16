#include "arctos_hardware/arctos_hardware.hpp"
#include <string>
#include <vector>
#include <vector>
#include <cmath>
#include <numeric>

#include <canary/frame_header.hpp>
#include <canary/interface_index.hpp>
#include <canary/raw.hpp>
#include <canary/socket_options.hpp>
#include <iostream>
#include <boost/asio/co_spawn.hpp>
#include <boost/asio/detached.hpp>
  
#include <iostream>
#include <vector>
#include <cstdint>

namespace arctos
{

int get_CRC(int id, const std::vector<uint8_t>& data) {
    int crc = id + std::accumulate(data.begin(), data.end(), 0);
    return crc & 0xFF;
}

bool check_CRC(int id, const std::array<std::uint8_t, 8> & data) {
    int sum = id;
    for (size_t i = 0; i < data.size() - 1; ++i) {
        sum += data[i];
    }
    return data.back() == (sum & 0xFF);
}

bool check_CRC_4(int id, const std::vector<std::uint8_t>  & data) {
    int sum = id;
    for (size_t i = 0; i < data.size() - 1; ++i) {
        sum += data[i];
    }
    return data.back() == (sum & 0xFF);
}

#include <iostream>
#include <vector>
#include <cstdint>

int64_t vectorToSignedInt46(const std::vector<std::uint8_t>& vec) {
    if (vec.size() != 6) { // Ensure the vector has 6 elements for 46-bit integer
        throw std::invalid_argument("Vector size must be 6 for a 46-bit integer.");
    }

    int64_t result = 0;
    for (size_t i = 0; i < vec.size(); ++i) {
        result = (result << 8) | static_cast<int64_t>(vec[i]);
    }

    if (result > 140737488355327){return result-281474976710656;}
    return result;
}

int16_t vectorToSignedInt16(const std::vector<std::uint8_t>& vec) {
    if (vec.size() != 2) { // Ensure the vector has 2 elements for 16-bit integer
        throw std::invalid_argument("Vector size must be 2 for a 16-bit integer.");
    }

    // Combine the bytes in big-endian order
    int16_t result = (static_cast<int16_t>(vec[0]) << 8) | static_cast<int16_t>(vec[1]);

    // If the sign bit (15th bit) is set, extend the sign bit to ensure it's interpreted as negative
    if (vec[0] & 0x80) { // Check if the sign bit is set (0x80 = 1000 0000)
        result |= static_cast<int16_t>(0xFFFF0000); // Extend the sign
    }

    return result;
}



CallbackReturn RobotSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // robot has 6 joints and 2 interfaces
  joint_position_.assign(6, 0);
  joint_velocities_.assign(6, 0);
  joint_position_command_.assign(6, 0);
  joint_velocities_command_.assign(6, 0);

  for (const auto & joint : info_.joints)
  {
    for (const auto & interface : joint.state_interfaces)
    {
      joint_interfaces[interface.name].push_back(joint.name);
    }
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RobotSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  
  int ind = 0;
  for (const auto & joint_name : joint_interfaces["position"])
  {
    state_interfaces.emplace_back(joint_name, "position", &joint_position_[ind++]);
  }

  ind = 0;
  for (const auto & joint_name : joint_interfaces["velocity"])
  {
    state_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_[ind++]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RobotSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  int ind = 0;
  for (const auto & joint_name : joint_interfaces["position"])
  {
    command_interfaces.emplace_back(joint_name, "position", &joint_position_command_[ind++]);
  }

  ind = 0;
  for (const auto & joint_name : joint_interfaces["velocity"])
  {
    command_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_command_[ind++]);
  }

  return command_interfaces;
}

return_type RobotSystem::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{


    /*

    //setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    std::vector<double> gear_ratios = {13.5,-150.,150.,48.,67.82,67.82};
    double encoder_resolution = 2*M_PI/(16384);

    boost::asio::io_context ioc;
    // Retrieve the interface index from the interface name
    const auto idx = canary::get_interface_index("can0");
    // Construct an endpoint using the index
    auto const ep = canary::raw::endpoint{idx};
    // Construct and bind a raw CAN frame socket to the endpoint.
    canary::raw::socket sock{ioc, ep};

    //Frame to get data.
    struct canFrame
    {
        canary::frame_header header;
        std::array<std::uint8_t, 8> payload;
    } cFrame;

    cFrame = {};

    for (int joint_id = 1; joint_id <= 4 ; joint_id++)
    {
      // Get position
      std::vector<uint8_t> command = {0x31};
      int crc = get_CRC(joint_id, command);
      command.push_back(static_cast<uint8_t>(crc));
      std::string command_str(reinterpret_cast<char*>(command.data()), command.size());
      cFrame.header.extended_format(false);

      cFrame.header.id(joint_id);
      cFrame.header.payload_length(boost::asio::buffer_copy(
        canary::net::buffer(cFrame.payload), boost::asio::buffer(command_str)));
      sock.send(canary::net::buffer(&cFrame, sizeof(cFrame)));
      
      // Add timeout
      sock.receive(canary::net::buffer(&cFrame, sizeof(cFrame)));

      if (check_CRC(joint_id,cFrame.payload) && static_cast<int>(cFrame.header.id()) == joint_id) 
      {
        std::vector<std::uint8_t> encoder_value_data = {cFrame.payload.begin()+1,cFrame.payload.end()-1};
        int64_t encoder_value = vectorToSignedInt46(encoder_value_data);
        double encoder_value_rad = encoder_value*encoder_resolution/gear_ratios[joint_id];
        joint_position_[joint_id-1] = encoder_value_rad;
        //std::cout<<"Joint " << joint_id << " rotation :" << encoder_value_rad << "\n";
      }

      // Get velocity
      std::vector<uint8_t> command2 = {0x32};
      int crc2 = get_CRC(joint_id, command2);
      command2.push_back(static_cast<uint8_t>(crc2));
      std::string command_str2(reinterpret_cast<char*>(command2.data()), command2.size());
      cFrame.header.extended_format(false);

      cFrame.header.id(joint_id);
      cFrame.header.payload_length(boost::asio::buffer_copy(
        canary::net::buffer(cFrame.payload), boost::asio::buffer(command_str2)));
      sock.send(canary::net::buffer(&cFrame, sizeof(cFrame)));
      
      // Add timeout
      sock.receive(canary::net::buffer(&cFrame, sizeof(cFrame)));

      std::vector<std::uint8_t> cFrame_payload_4 = {cFrame.payload.begin(),cFrame.payload.begin()+4};
      if (check_CRC_4(joint_id,cFrame_payload_4)) 
      {
        int16_t velocity_value = (static_cast<int16_t>(cFrame_payload_4[1]) << 8) | cFrame_payload_4[2];
        double velocity_value_rad = velocity_value*2*M_PI/60/gear_ratios[joint_id];
        joint_velocities_[joint_id-1] = velocity_value_rad;
      }
      
  }

*/

  for (auto i = 0; i < joint_velocities_command_.size(); i++)
  {
    joint_velocities_[i] = joint_velocities_command_[i];
    joint_position_[i] += joint_velocities_command_[i] * period.seconds();
  }

  for (auto i = 0; i < joint_position_command_.size(); i++)
  {
    joint_position_[i] = joint_position_command_[i];
  }

  return return_type::OK;
}

return_type RobotSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  return return_type::OK;
}

}  // namespace ros2_control_demo_example_7

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  arctos::RobotSystem, hardware_interface::SystemInterface)