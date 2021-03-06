#include "bus_servo.h"

#include <iostream>
#include <string>

using namespace std;
int main(int argc, char ** argv) {
  cout << "This is bus servo config" << endl;
  string usb_port_name = "/dev/servo-bus";
  int serial_port = open(usb_port_name.c_str(), O_RDWR);
  config_serial_port(serial_port);


  // list all connected servos
  for(uint8_t servo_id=0;servo_id<255;++servo_id) {
    uint8_t returned_servo_id;
    bool ok = servo_id_read(serial_port, servo_id, &returned_servo_id);
    if(ok) {
      cout << "found a servo at servo_id " << (int)servo_id << endl;
    }
  }

  // set the servo id
  string new_servo_id_string;
  cout << "what servo id would you like to set: " << flush;

  cin >> new_servo_id_string;
  uint8_t new_servo_id = atoi(new_servo_id_string.c_str());
  cout << "setting servo id to " << (int)new_servo_id << endl;

  // setting servo id to 

  servo_id_write(serial_port, 254, new_servo_id);

  

  return 0;
}