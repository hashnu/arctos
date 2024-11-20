#include "arctos_hardware/arctos_hardware.hpp"
#include <string>
#include <vector>
#include <cmath>
#include <numeric>
#include <iostream>
#include <cstdint>

#include <canary/frame_header.hpp>
#include <canary/interface_index.hpp>
#include <canary/raw.hpp>
#include <canary/socket_options.hpp>
#include <boost/asio/co_spawn.hpp>
#include <boost/asio/detached.hpp>
  
namespace arctos
{

const std::vector<double> gear_ratios = {13.5,-150.,150.,48.,67.82,67.82};
const double encoder_resolution = 2*M_PI/(16384);
const double joint_rotation_speed_rpm = 2./5;
const uint8_t motor_acceleration = 1;
const int can_timeout = 100;

int get_CRC(int id, const std::vector<uint8_t>& data) {
    int crc = id + std::accumulate(data.begin(), data.end(), 0);
    return crc & 0xFF;
}

bool check_CRC(int id, const std::vector<std::uint8_t> & data) {
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

std::vector<uint8_t> int32ToByteArray(int32_t value) {
    std::vector<uint8_t> byteArray(4); // 4 bytes for a 32-bit integer

    //if (value<0) {value = -value+8388607;}

    // Store each byte in big-endian order
    byteArray[0] = static_cast<uint8_t>((value >> 16) & 0xFF);
    byteArray[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
    byteArray[2] = static_cast<uint8_t>(value & 0xFF);

    return byteArray;
}

std::vector<uint8_t> motorSpeedToByteArray(double speed) {
    std::vector<uint8_t> byteArray(2);

    if (speed<0) {speed = -speed;}
    if (speed>3000) {speed=3000.;}
    
    int32_t speed_int = std::round(speed);

    byteArray[0] = static_cast<uint8_t>((speed_int >> 8) & 0xFF);
    byteArray[1] = static_cast<uint8_t>(speed_int & 0xFF);

    return byteArray;
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
    uint32_t bit_mask = 0x01;

    for ( int joint_id = 1; joint_id <= 3 ; joint_id++)
    {

      sock.set_option(canary::filter_if_any{{{
      canary::filter{}
        .id_mask(~joint_id)            // Only Joint ID.
        //.remote_transmission(false) // Only remote transmission frames.
        .extended_format(false)    // Only standard format frames.
      }}});

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
    
      while (true) {        
        sock.receive(canary::net::buffer(&cFrame, sizeof(cFrame)));

        std::vector<std::uint8_t> cFrame_payload(cFrame.payload.begin(), cFrame.payload.end());

        if (check_CRC(joint_id,cFrame_payload))
        {
          if (cFrame.header.payload_length() == 8 && static_cast<int>(cFrame.payload[0]) == 49)
          {
            std::vector<std::uint8_t> encoder_value_data = {cFrame.payload.begin()+1,cFrame.payload.end()-1};
            int64_t encoder_value = vectorToSignedInt46(encoder_value_data);
            double encoder_value_rad = encoder_value*encoder_resolution/gear_ratios[joint_id-1];
            joint_position_[joint_id-1] = encoder_value_rad;
            break;
            //std::cout<<"Joint " << joint_id << " rotation :" << encoder_value_rad << "\n";
          }
        }
        break;
      }
        //} else {break;}
        //std::this_thread::sleep_for(std::chrono::microseconds(10));
      //}

      // Get velocity
      std::vector<uint8_t> command2 = {0x32};
      int crc2 = get_CRC(joint_id, command2);
      command2.push_back(static_cast<uint8_t>(crc2));
      std::string command_str2(reinterpret_cast<char*>(command2.data()), command2.size());
      cFrame.header.extended_format(false);

      cFrame.header.id(joint_id);
      cFrame.header.payload_length(boost::asio::buffer_copy(canary::net::buffer(cFrame.payload), boost::asio::buffer(command_str2)));
      sock.send(canary::net::buffer(&cFrame, sizeof(cFrame)));
      
      while (true) {        
          sock.receive(canary::net::buffer(&cFrame, sizeof(cFrame)));

          std::vector<std::uint8_t> cFrame_payload(cFrame.payload.begin(), cFrame.payload.end());

          if (check_CRC(joint_id,cFrame_payload))
          {
            if (cFrame.header.payload_length() == 4 && static_cast<int>(cFrame.payload[0]) == 49)
            {
              int16_t velocity_value = (static_cast<int16_t>(cFrame_payload[1]) << 8) | cFrame_payload[2];
              double velocity_value_rad = velocity_value*2*M_PI/60/gear_ratios[joint_id-1];
              joint_velocities_[joint_id-1] = velocity_value_rad;
              break;
              //std::cout<<"Joint " << joint_id << " rotation :" << encoder_value_rad << "\n";
            }
          } else {break;}
          std::this_thread::sleep_for(std::chrono::microseconds(10));
      }      
  }

  for (auto i = 3; i < joint_velocities_command_.size(); i++)
  {
    joint_velocities_[i] = joint_velocities_command_[i];
    joint_position_[i] += joint_velocities_command_[i] * period.seconds();
  }

  for (auto i = 3; i < joint_position_command_.size(); i++)
  {
    joint_position_[i] = joint_position_command_[i];
  }

  return return_type::OK;
}

return_type RobotSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{

  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  //std::cout << "X joint command:" << joint_position_command_[0] << "\n";
  //std::vector<double> gear_ratios = {13.5,150.,150.,48.,67.82,67.82};
  //double encoder_resolution = 2*M_PI/(16384);

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

  if (joint_position_command_.size() > 0) {
    for (int joint_id = 1; joint_id <= 3;joint_id++)//joint_position_command_.size(); i++)
    {

      double encoder_value_double = -joint_position_command_[joint_id-1]*gear_ratios[joint_id-1]/encoder_resolution;
      if (joint_id==3) { encoder_value_double = -encoder_value_double;}
      int32_t encoder_int = static_cast<int32_t>(std::round(encoder_value_double));
      std::vector<uint8_t> encoder_byte = int32ToByteArray(encoder_int);

      double motor_speed_rpm = joint_rotation_speed_rpm*gear_ratios[joint_id-1];
      std::vector<uint8_t> motor_speed = motorSpeedToByteArray(motor_speed_rpm);

      std::vector<uint8_t> command = {0xF5,motor_speed[0],motor_speed[1],motor_acceleration,encoder_byte[0],encoder_byte[1],encoder_byte[2]};
      command.push_back(get_CRC(joint_id, command));

      std::string command_str(reinterpret_cast<char*>(command.data()), command.size());
      
      cFrame.header.id(joint_id);
      cFrame.header.payload_length(boost::asio::buffer_copy(canary::net::buffer(cFrame.payload), boost::asio::buffer(command_str)));
      sock.send(canary::net::buffer(&cFrame, sizeof(cFrame)));

      /*
      if (joint_id == 1) {
      std::cout << "Command: ";
      std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(command[0]) << " - ";
      std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(command[1]) << " - ";
      std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(command[2]) << " - ";
      std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(command[3]) << " - ";
      std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(command[4]) << " - ";
      std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(command[5]) << " - ";
      std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(command[6]) ;
      std::cout << "\n";}
      */


  }
  
  }
  

  return return_type::OK;
}

}  // namespace ros2_control_demo_example_7

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  arctos::RobotSystem, hardware_interface::SystemInterface)