#include "bus_servo.h"

#include "rclcpp/rclcpp.hpp"
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include "std_msgs/msg/float64.hpp"
#include <string>
#include <sstream>
#include <vector>
#include <chrono>

#include "bus_servo_interfaces/msg/servo_command.hpp"


// handles a single servo
class RosBusServo {
  public:
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher;

  int servo_id = 0;
  int serial_port = 0;
  rclcpp::Node::SharedPtr node;
  rclcpp::Subscription<bus_servo_interfaces::msg::ServoCommand>::SharedPtr sub;
  bus_servo_interfaces::msg::ServoCommand command;


  int16_t position;

  void command_callback(const bus_servo_interfaces::msg::ServoCommand::SharedPtr msg) {
    auto &cmd = *msg.get();

    if(std::isnan( cmd.angle) || cmd.max_vel == 0) {
      servo_load_or_unload_write(serial_port, servo_id, 0);
      return;
    }

    int16_t move_pos = cmd.angle;
    uint16_t move_ms = cmd.max_vel > 0 ? fabs((position-cmd.angle)/cmd.max_vel)*1000 : 0;
    servo_move_time_write(serial_port, servo_id, cmd.angle, move_ms);
  }     
  

  void init(int serial_port, int servo_id, rclcpp::Node::SharedPtr node) {
    this->serial_port = serial_port;
    this->servo_id = servo_id;
    this->node = node;
    std::string topic = "/servo" + to_string(servo_id);
    RCLCPP_INFO(node->get_logger(), "publishing on %s",topic.c_str());
    publisher = node->create_publisher<std_msgs::msg::Float64>(topic, 10);
    string cmd_topic = topic+"_cmd";
    
    sub = node->create_subscription<bus_servo_interfaces::msg::ServoCommand>(
      cmd_topic, 1, std::bind(&RosBusServo::command_callback, this, std::placeholders::_1) );

    RCLCPP_INFO(node->get_logger(), "listening on %s",cmd_topic.c_str());
    
  }


  void loop() {
    std_msgs::msg::Float64 msg;
    if(servo_pos_read(serial_port, servo_id, &position)) {
      msg.data = position;
    } else {
      msg.data = NAN;
      // ROS_DEBUG_THROTTLE(60, "failed to read servo %d", servo_id);
    }
    publisher->publish(msg);
  }


  // adds status for this servo to diag array
  void get_status(std::string parent_component, diagnostic_msgs::msg::DiagnosticArray & diag_array) {
      diagnostic_msgs::msg::DiagnosticStatus servo_status;
      servo_status.name = parent_component+": Servo " + std::to_string(servo_id);

      servo_status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      servo_status.message = "ok";

      diagnostic_msgs::msg::KeyValue current_position_kv;
      int16_t position;
      if(servo_pos_read(serial_port, servo_id, &position)) {
        current_position_kv.key = "Current Position";
        current_position_kv.value = std::to_string(position);
      };
      servo_status.values.push_back(current_position_kv);


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

int run(int argc, char** argv) {
    std::vector<int> servo_ids = {1,2};

    // initialize ros
    std::string node_name = "bus_servo_node";
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("node_name");
    auto diagnostic_pub = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 1);

    // open serial port
    std::string port_path = "/dev/servo-bus"; 
    int serial_port = open(port_path.c_str(), O_RDWR);

    if(serial_port == -1) {
      fail((std::string)"failed to open serial port " + port_path);
    } 
    cout << "opened serial port " << port_path << " as " << serial_port << endl;

    config_serial_port(serial_port);


    // initialize servos    
    std::vector<RosBusServo> servos;
    servos.resize(servo_ids.size());
    for(int i=0; i<servo_ids.size(); ++i) {
      servos[i].init(serial_port, servo_ids[i], node);
    }

    int loop_hz = 100;
    int64_t loop_count = 0;
    rclcpp::Rate loop_rate(loop_hz);
    while(rclcpp::ok()) {
      ++loop_count;
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
          servo.get_status(node_name, diag_array);
        }

        diagnostic_pub->publish(diag_array);

      }
      rclcpp::spin_some(node);
      loop_rate.sleep();
    }
    close(serial_port);
    return 0;
}

int main(int argc, char** argv) {
  try {
    return run(argc, argv);
  }
  catch(string s) {
    cout << "Failed, error caught in main: " << s << endl;
  }
  return 0;
}
