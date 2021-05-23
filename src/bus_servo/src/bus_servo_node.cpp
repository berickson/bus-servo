#include "bus_servo.h"

#include <ros/ros.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include "std_msgs/Float64.h"
#include <string>
#include <sstream>
#include <vector>
#include <chrono>

#include "bus_servo/ServoCommand.h"


// handles a single servo
class RosBusServo {
  public:
  ros::Publisher publisher;
  ros::NodeHandle nh;

  int servo_id = 0;
  int serial_port = 0;
  ros::Publisher pub;
  ros::Subscriber sub;

  int16_t position;

  void command_callback(const bus_servo::ServoCommand::ConstPtr& msg) {
    auto &cmd = *msg.get();

    if(std::isnan( cmd.angle) && std::isnan(cmd.max_vel)) {
      servo_load_or_unload_write(serial_port, servo_id, 0);
      return;
    }

    int16_t move_pos = cmd.angle;
    uint16_t move_ms = cmd.max_vel > 0 ? fabs((position-cmd.angle)/cmd.max_vel)*1000 : 0;
    servo_move_time_write(serial_port, servo_id, cmd.angle, move_ms);
  }
  

  void init(int serial_port, int servo_id) {
    this->serial_port = serial_port;
    this->servo_id = servo_id;
    std::string topic = "/servo" + to_string(servo_id);
    ROS_INFO("publishing on %s",topic.c_str());
    auto publisher = nh.advertise<std_msgs::Float64>(topic, 10);
    string cmd_topic = topic+"_cmd";
    sub = nh.subscribe(cmd_topic, 1, &RosBusServo::command_callback, this);
    ROS_INFO("listening on %s",cmd_topic.c_str());
    
    pub = publisher;
  }


  void loop() {
    if(servo_pos_read(serial_port, servo_id, &position)) {
      std_msgs::Float64 msg;
      msg.data = position;
      pub.publish(msg);
    } else {
      // ROS_DEBUG_THROTTLE(60, "failed to read servo %d", servo_id);
    }
  }


  // adds status for this servo to diag array
  void get_status(std::string parent_component, diagnostic_msgs::DiagnosticArray & diag_array) {
      diagnostic_msgs::DiagnosticStatus servo_status;
      servo_status.name = parent_component+": Servo " + std::to_string(servo_id);

      servo_status.level = diagnostic_msgs::DiagnosticStatus::OK;
      servo_status.message = "ok";


      diagnostic_msgs::KeyValue voltage_kv;
      voltage_kv.key = "Voltage";
      uint16_t vin;
      if(servo_vin_read(serial_port, servo_id, &vin)) {
        voltage_kv.value = std::to_string(int(vin));
      }
      servo_status.values.push_back(voltage_kv);

      diagnostic_msgs::KeyValue vin_min_kv, vin_max_kv;
      uint16_t vin_min, vin_max;
     
      if(servo_vin_limit_read(serial_port, servo_id, &vin_min, &vin_max)) {
        vin_min_kv.key = "Voltage Min Limit";
        vin_min_kv.value = std::to_string(vin_min);
        servo_status.values.push_back(vin_min_kv);

        vin_max_kv.key = "Voltage Max Limit";
        vin_max_kv.value = std::to_string(vin_max);
        servo_status.values.push_back(vin_max_kv);

        if(vin < vin_min) {
          servo_status.level = diagnostic_msgs::DiagnosticStatus::WARN;
          servo_status.message = "Low Voltage Detected";
        }

        if(vin > vin_max) {
          servo_status.level = diagnostic_msgs::DiagnosticStatus::WARN;
          servo_status.message = "Over Voltage Detected";
        }
      }

      uint8_t temp,temp_max;
      diagnostic_msgs::KeyValue kv;
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
          servo_status.level = diagnostic_msgs::DiagnosticStatus::WARN;
          servo_status.message = "Over Temp Detected";
      }
    

      uint8_t load_mode;
      if(servo_load_or_unload_read(serial_port, servo_id, &load_mode)) {
        diagnostic_msgs::KeyValue kv;

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
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::Publisher diagnostic_pub = nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1);

    // open serial port
    int serial_port = open("/dev/servo-bus", O_RDWR);
    config_serial_port(serial_port);


    // initialize servos    
    std::vector<RosBusServo> servos;
    servos.resize(servo_ids.size());
    for(int i=0; i<servo_ids.size(); ++i) {
      servos[i].init(serial_port, servo_ids[i]);
    }

    int loop_hz = 100;
    int64_t loop_count = 0;
    ros::Rate loop_rate(loop_hz);
    while(ros::ok()) {
      ++loop_count;
      for(auto & servo : servos) {
        servo.loop();
      }

      // occasionally publish statistics
      // see https://answers.ros.org/question/262236/publishing-diagnostics-from-c/
      if(loop_count%loop_hz == 0) {
        diagnostic_msgs::DiagnosticArray diag_array;
        diagnostic_msgs::DiagnosticStatus bus_status;
        bus_status.name = node_name +": Servo Bus";
        bus_status.level = diagnostic_msgs::DiagnosticStatus::OK;
        bus_status.message = "ok";
        diagnostic_msgs::KeyValue loop_count_kv;
        loop_count_kv.key = "loop count";
        loop_count_kv.value = std::to_string(loop_count);
        bus_status.values.push_back(loop_count_kv);
        diag_array.status.push_back(bus_status);

        for(auto & servo : servos) {
          servo.get_status(node_name, diag_array);
        }

        diagnostic_pub.publish(diag_array);

      }
      ros::spinOnce();
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
