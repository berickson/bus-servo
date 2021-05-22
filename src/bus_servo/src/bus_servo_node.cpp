#include "bus_servo.h"

#include <ros/ros.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include "std_msgs/Float32.h"
#include <string>
#include <sstream>
#include <vector>
#include <chrono>


// handles a single servo
class RosBusServo {
  public:
  ros::Publisher publisher;
  ros::NodeHandle nh;

  int servo_id = 0;
  int serial_port = 0;
  ros::Publisher pub;

  void init(int serial_port, int servo_id) {
    this->serial_port = serial_port;
    this->servo_id = servo_id;
    std::stringstream topic;
    topic << "bus_servos/" << servo_id;
    auto publisher = nh.advertise<std_msgs::Float32>(topic.str().c_str(), 10);
    
    pub = publisher;
  }
  void loop() {
    int16_t position;
    if(servo_pos_read(serial_port, servo_id, &position)) {
      std_msgs::Float32 msg;
      msg.data = position;
      pub.publish(msg);
    } else {
      // ROS_DEBUG_THROTTLE(60, "failed to read servo %d", servo_id);
    }
  }

};

int run(int argc, char** argv) {
    std::vector<int> servo_ids = {1,2};

    // initialize ros
    ros::init(argc, argv, "bus_servo_node");
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
    int loop_count;    
    ros::Rate loop_rate(loop_hz);
    while(ros::ok()) {
      ++loop_count;
      for(auto & servo : servos) {
        servo.loop();
      }

      // occasionally publish statistics
      // see https://answers.ros.org/question/262236/publishing-diagnostics-from-c/
      if(loop_count%loop_hz == 0) {
        diagnostic_msgs::DiagnosticArray dia_array;
        diagnostic_msgs::DiagnosticStatus robot_status;
        robot_status.name = "Servo Bus";
        robot_status.level = diagnostic_msgs::DiagnosticStatus::OK;
        robot_status.message = "ok";
        diagnostic_msgs::KeyValue loop_count_kv;
        loop_count_kv.key = "loop count";
        loop_count_kv.value = std::to_string(loop_count);
        robot_status.values.push_back(loop_count_kv);

        dia_array.status.push_back(robot_status);
        diagnostic_pub.publish(dia_array);

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
