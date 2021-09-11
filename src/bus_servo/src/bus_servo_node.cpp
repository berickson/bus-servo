#include "bus_servo.h"

#include "rclcpp/rclcpp.hpp"
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/integer_range.hpp"
#include <string>
#include <sstream>
#include <vector>
#include <chrono>

#include "bus_servo_interfaces/msg/servo_command.hpp"
#include "bus_servo_interfaces/srv/servo_move_time_write.hpp"

std::mutex serial_mutex;

// handles a single servo
class RosBusServo {
  public:
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher;

  int servo_id = 0;
  int serial_port = 0;
  int64_t ok_read_count = 0;
  int64_t failed_read_count = 0;
  rclcpp::Node::SharedPtr node;
  string name;
  string fixed_name;
  rclcpp::Subscription<bus_servo_interfaces::msg::ServoCommand>::SharedPtr sub;
  bus_servo_interfaces::msg::ServoCommand command;
  rclcpp::Service<bus_servo_interfaces::srv::ServoMoveTimeWrite>::SharedPtr servo_move_time_write_service;


  int16_t position;

  static std::mutex callback_mutex;
  void command_callback(const bus_servo_interfaces::msg::ServoCommand::SharedPtr msg) {
    // const std::lock_guard<std::mutex> lock(callback_mutex);
    auto &cmd = *msg.get();

    if(std::isnan( cmd.angle)) {
      //servo_load_or_unload_write(serial_port, servo_id, 0);
      return;
    } else {

      uint16_t move_ms = cmd.max_vel == 0 ? 0 : fabs((position-cmd.angle)/cmd.max_vel)*1000;
      servo_move_time_write(serial_port, servo_id, cmd.angle, move_ms);
      int16_t out_position;
      uint16_t out_ms;

      servo_move_time_read(serial_port, servo_id, &out_position, &out_ms);
    }
    // cout << "servo " << servo_id << " commanded to " << cmd.angle << " on port " << serial_port << endl;
  }

  void servo_move_time_write_callback(
      const std::shared_ptr<bus_servo_interfaces::srv::ServoMoveTimeWrite_Request> request,
      std::shared_ptr<bus_servo_interfaces::srv::ServoMoveTimeWrite_Response> /* response */)
  {
    servo_move_time_write(serial_port, servo_id, request->position , request->ms);
  }

  

  void init(int serial_port, string fixed_name, string name, int servo_id,rclcpp::Node::SharedPtr node) {
    this->name = name;
    this->fixed_name = fixed_name;
    this->serial_port = serial_port;
    this->servo_id = servo_id;
    this->node = node;
    std::string topic = "/servo/"+ name;
    RCLCPP_INFO(node->get_logger(), "publishing on %s",topic.c_str());
    publisher = node->create_publisher<std_msgs::msg::Float64>(topic, 10);
    std::string cmd_topic = "/servo_cmd/" + name;

    
    sub = node->create_subscription<bus_servo_interfaces::msg::ServoCommand>(
      cmd_topic, 1, std::bind(&RosBusServo::command_callback, this, std::placeholders::_1) );

    servo_move_time_write_service = node->create_service<bus_servo_interfaces::srv::ServoMoveTimeWrite>(
      topic + "/servo_move_time_write",  
      std::bind(&RosBusServo::servo_move_time_write_callback, this, std::placeholders::_1, std::placeholders::_2 ));

    RCLCPP_INFO(node->get_logger(), "listening on %s",cmd_topic.c_str());
    
  }


  void loop() {
    std_msgs::msg::Float64 msg;

    if(servo_pos_read(serial_port, servo_id, &position)) {
      msg.data = position;
      ++ok_read_count;
    } else {
        ++failed_read_count;
        msg.data = NAN;
        rclcpp::Clock clock;
        RCLCPP_DEBUG(node->get_logger(), "failed to read servo %d position", servo_id);
    }
    if(!isnan(msg.data)) {
      publisher->publish(msg);
    }
  }


  // adds status for this servo to diag array
  void get_diagnostics(std::string parent_component, diagnostic_msgs::msg::DiagnosticArray & diag_array) {
      diagnostic_msgs::msg::DiagnosticStatus servo_status;
      servo_status.name = parent_component+":  " + fixed_name +"("+this->name+")";

      servo_status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      servo_status.message = "ok";

      diagnostic_msgs::msg::KeyValue current_position_kv;
      int16_t position;
      if(servo_pos_read(serial_port, servo_id, &position)) {
        current_position_kv.key = "Current Position";
        current_position_kv.value = std::to_string(position);
      } else {
        servo_status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        servo_status.message = "communication failure";
      }
      servo_status.values.push_back(current_position_kv);


      diagnostic_msgs::msg::KeyValue ok_read_count_msg;
      ok_read_count_msg.key = "OK Read Count";
      ok_read_count_msg.value = std::to_string(ok_read_count);
      servo_status.values.push_back(ok_read_count_msg);

      diagnostic_msgs::msg::KeyValue failed_read_count_msg;
      failed_read_count_msg.key = "Failed Read Count";
      failed_read_count_msg.value = std::to_string(failed_read_count);
      servo_status.values.push_back(failed_read_count_msg);


      diagnostic_msgs::msg::KeyValue position_min_kv;
      diagnostic_msgs::msg::KeyValue position_max_kv;

      position_min_kv.key = "Position Min Limit";
      position_max_kv.key = "Position Max Limit";
      uint16_t min_position, max_position;
      if(servo_angle_limit_read(serial_port, servo_id, &min_position, &max_position)) {
        position_min_kv.value = std::to_string(int(min_position));
        position_max_kv.value = std::to_string(int(max_position));

      }
      servo_status.values.push_back(position_min_kv);
      servo_status.values.push_back(position_max_kv);





      diagnostic_msgs::msg::KeyValue voltage_kv;
      voltage_kv.key = "Voltage";
      uint16_t vin;
      if(servo_vin_read(serial_port, servo_id, &vin)) {
        voltage_kv.value = std::to_string(int(vin));
      }
      servo_status.values.push_back(voltage_kv);

          diagnostic_msgs::msg::KeyValue vin_min_kv, vin_max_kv;
      uint16_t vin_min, vin_max;
     
      if(servo_vin_limit_read(serial_port, servo_id, &vin_min, &vin_max)) {
        vin_min_kv.key = "Voltage Min Limit";
        vin_min_kv.value = std::to_string(vin_min);
        servo_status.values.push_back(vin_min_kv);

        vin_max_kv.key = "Voltage Max Limit";
        vin_max_kv.value = std::to_string(vin_max);
        servo_status.values.push_back(vin_max_kv);

        if(vin < vin_min) {
          servo_status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
          servo_status.message = "Low Voltage Detected";
        }

        if(vin > vin_max) {
          servo_status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
          servo_status.message = "Over Voltage Detected";
        }
      }

      uint8_t temp,temp_max;
      diagnostic_msgs::msg::KeyValue kv;
      if(servo_temp_read(serial_port, servo_id, &temp)) {
        kv.key = "Temp";
        kv.value = std::to_string(temp);
        servo_status.values.push_back(kv);
      }
      if(servo_temp_max_limit_read(serial_port, servo_id, &temp_max)) {
        kv.key = "Temp Max Limit";
        kv.value = std::to_string(temp_max);
        servo_status.values.push_back(kv);
      }

      if(temp>temp_max) {
          servo_status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
          servo_status.message = "Over Temp Detected";
      }
    

      uint8_t load_mode;
      if(servo_load_or_unload_read(serial_port, servo_id, &load_mode)) {
        diagnostic_msgs::msg::KeyValue kv;

        kv.key = "Load Mode";
        kv.value = load_mode ? "Powered" : "Unpowered";
        servo_status.values.push_back(kv);
      }


      diag_array.status.push_back(servo_status);
  }

};


class BusServoNode {
  public:
  rclcpp::Node::SharedPtr node_;
  bool param_update_pending_ = false;
  std::vector<RosBusServo> servos;  
  int serial_port = -1;

  // see https://roboticsbackend.com/ros2-rclcpp-parameter-callback/
  rcl_interfaces::msg::SetParametersResult parameters_callback(const std::vector<rclcpp::Parameter> parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    RCLCPP_INFO(node_->get_logger(), "parameters callback");
    result.successful = true;
    result.reason = "success";

    param_update_pending_ = true;
    return result;
  }

  void update_from_parameters() {

    // add or remove parameters based on servo_count
    auto old_servo_count = servos.size();
    auto new_servo_count = node_->get_parameter("servo_count").as_int();

    for(int i = old_servo_count+1; i <= new_servo_count; ++i) {
      string fixed_name = "servo"+to_string(i);
      cout << "adding " << fixed_name << endl;
      node_->declare_parameter(fixed_name+"/name","servo"+to_string(i));

      rcl_interfaces::msg::ParameterDescriptor id_descriptor;
      rcl_interfaces::msg::IntegerRange id_range;
      id_range.set__from_value(0).set__to_value(253).set__step(1);
      id_descriptor.integer_range= {id_range};
      id_descriptor.description = "Internal address of the servo to control";

      node_->declare_parameter(fixed_name+"/id", i, id_descriptor);
    }
    for(int i = new_servo_count+1; i <= old_servo_count; ++i) {
      string fixed_name = "servo"+to_string(i);
      cout << "adding " << fixed_name << endl;
      node_->undeclare_parameter(fixed_name+"/name");
      node_->undeclare_parameter(fixed_name+"/id");
    }

    // initialize servos
    servos.resize(new_servo_count);
    for(uint i=0; i<servos.size(); ++i) {
      string fixed_name = "servo"+to_string(i+1);
      auto id = node_->get_parameter(fixed_name+"/id").as_int();
      string name = node_->get_parameter(fixed_name+"/name").as_string();
      
      servos[i].init(serial_port, fixed_name, name, id, node_);
    }

      param_update_pending_ = false;
  }

  int run(int argc, char** argv) {

      // initialize ros
      std::string node_name = "bus_servo_node";
      rclcpp::init(argc, argv);
      auto node = rclcpp::Node::make_shared(node_name);
      node_ = node;
      auto diagnostic_pub = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 1);


      rcl_interfaces::msg::ParameterDescriptor descriptor;
      rcl_interfaces::msg::IntegerRange range;
      range.set__from_value(0).set__to_value(100).set__step(1);
      descriptor.integer_range= {range};

      node->declare_parameter("servo_count", 2, descriptor);

      

      cout << "registering callback" << endl;
      auto callback_handle = node->add_on_set_parameters_callback(std::bind(&BusServoNode::parameters_callback, this, std::placeholders::_1));
      cout << "registered callback" << endl;

      // auto param_subscriber = std::make_shared<rclcpp::ParameterEventHandler>(node);


      // open serial port
      std::string port_path = "/dev/servo-bus"; 
      serial_port = open(port_path.c_str(), O_RDWR);

      if(serial_port == -1) {
        fail((std::string)"failed to open serial port " + port_path);
      } 
      cout << "opened serial port " << port_path << " as " << serial_port << endl;

      config_serial_port(serial_port);


      update_from_parameters();


      int loop_hz = 100;
      int64_t loop_count = 0;
      rclcpp::Rate loop_rate(loop_hz);
      while(rclcpp::ok()) {
        ++loop_count;

        if(param_update_pending_) {
          update_from_parameters();
        }

        for(auto & servo : servos) {
          servo.loop();
        }

        // occasionally publish statistics
        // see https://answers.ros.org/question/262236/publishing-diagnostics-from-c/
        if(loop_count%loop_hz == 0) {
          diagnostic_msgs::msg::DiagnosticArray diag_array;
          diagnostic_msgs::msg::DiagnosticStatus bus_status;
          bus_status.name = node_name +": Servo Bus";
          bus_status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
          bus_status.message = "ok";
          diagnostic_msgs::msg::KeyValue loop_count_kv;
          loop_count_kv.key = "loop count";
          loop_count_kv.value = std::to_string(loop_count);
          bus_status.values.push_back(loop_count_kv);
          diag_array.status.push_back(bus_status);

          for(auto & servo : servos) {
            servo.get_diagnostics(node_name, diag_array);
          }

          diag_array.header.stamp = rclcpp::Clock().now();

          diagnostic_pub->publish(diag_array);
        }
        try {
        rclcpp::spin_some(node);
        } catch (rclcpp::exceptions::RCLError & e) {
          cout << "RCLError caught: " << e.what() << endl;
        }

        loop_rate.sleep();
      }
      close(serial_port);
      return 0;
  }
};

int main(int argc, char** argv) {
  try {
    BusServoNode node;
    return node.run(argc, argv);
  }
  catch(string s) {
    cout << "Failed, error caught in main: " << s << endl;
  }
  return 0;
}
