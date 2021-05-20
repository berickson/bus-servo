#include <iostream>
#include <fstream>

#include <string>
#include <thread>
#include <chrono>

#include "string.h"

#include <stdio.h>
#include <string.h>
#include <cmath>

#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()


using namespace std;

typedef char byte;

#define GET_LOW_BYTE(A) (uint8_t)((A))
//Macro function  get lower 8 bits of A
#define GET_HIGH_BYTE(A) (uint8_t)((A) >> 8)
//Macro function  get higher 8 bits of A
#define BYTE_TO_HW(A, B) ((((uint16_t)(A)) << 8) | (uint8_t)(B))

#define LOBOT_SERVO_FRAME_HEADER         0x55
#define LOBOT_SERVO_MOVE_TIME_WRITE      1
#define LOBOT_SERVO_MOVE_TIME_READ       2
#define LOBOT_SERVO_MOVE_TIME_WAIT_WRITE 7
#define LOBOT_SERVO_MOVE_TIME_WAIT_READ  8
#define LOBOT_SERVO_MOVE_START           11
#define LOBOT_SERVO_MOVE_STOP            12
#define LOBOT_SERVO_ID_WRITE             13
#define LOBOT_SERVO_ID_READ              14
#define LOBOT_SERVO_ANGLE_OFFSET_ADJUST  17
#define LOBOT_SERVO_ANGLE_OFFSET_WRITE   18
#define LOBOT_SERVO_ANGLE_OFFSET_READ    19
#define LOBOT_SERVO_ANGLE_LIMIT_WRITE    20
#define LOBOT_SERVO_ANGLE_LIMIT_READ     21
#define LOBOT_SERVO_VIN_LIMIT_WRITE      22
#define LOBOT_SERVO_VIN_LIMIT_READ       23
#define LOBOT_SERVO_TEMP_MAX_LIMIT_WRITE 24
#define LOBOT_SERVO_TEMP_MAX_LIMIT_READ  25
#define LOBOT_SERVO_TEMP_READ            26
#define LOBOT_SERVO_VIN_READ             27
#define LOBOT_SERVO_POS_READ             28
#define LOBOT_SERVO_OR_MOTOR_MODE_WRITE  29
#define LOBOT_SERVO_OR_MOTOR_MODE_READ   30
#define LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE 31
#define LOBOT_SERVO_LOAD_OR_UNLOAD_READ  32
#define LOBOT_SERVO_LED_CTRL_WRITE       33
#define LOBOT_SERVO_LED_CTRL_READ        34
#define LOBOT_SERVO_LED_ERROR_WRITE      35
#define LOBOT_SERVO_LED_ERROR_READ       36

byte LobotCheckSum(byte buf[])
{
  byte i;
  uint16_t temp = 0;
  for (i = 2; i < buf[3] + 2; i++) {
    temp += buf[i];
  }
  temp = ~temp;
  i = (byte)temp;
  return i;
}

void fail(std::string s) {
  throw s;
}

void clear_serial_input(int serial_port) {
    // empty anything on the serial bus;
  char trash[1000];
  read(serial_port, &trash, sizeof(trash));
}

int LobotSerialServoMoveTimeWrite(int serial_port, uint8_t id, uint16_t position, uint16_t ms)
{
  
  int count = 10000;
  int ret;
  byte buf[10];

  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 7;
  buf[4] = LOBOT_SERVO_MOVE_TIME_WRITE;
  buf[5] = GET_LOW_BYTE(position); // lower 8 bits of angle
  buf[6] = GET_HIGH_BYTE(position); // higher 8 bits of angle
  buf[7] = GET_LOW_BYTE(ms); // lower 8 bits of time in ms (0-3000)
  buf[8] = GET_HIGH_BYTE(ms);  // higher 8 bits of time 
  buf[9] =   LobotCheckSum(buf);

  // empty anything on the serial bus;
  clear_serial_input(serial_port);

  write(serial_port, buf, 10);

  return 0;
}


// returns true on success
bool read_with_timeout(int serial_port, char* buf, int len, int timeout_ms = 500) {
  auto t1 = std::chrono::high_resolution_clock::now();


  while(true) {
    int i = read(serial_port, buf, len);
    if(i > 0) {
      buf+=i;
      len-=i;
    }
    if(len == 0) {
      return true;
    }
    auto t2 = std::chrono::high_resolution_clock::now();
    if(std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() > timeout_ms) {
      cout << "read timed out" << endl;
      return false;
    }
    std::this_thread::sleep_for(std::chrono::microseconds(10));
  }
}

// load 0 is unpowered, 1 is powered
void LobotServoLoadOrUnloadWrite(int serial_port, uint8_t id, char load){
  char buf[7];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 4;
  buf[4] = LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE;
  buf[5] = load;
  buf[6] = LobotCheckSum(buf);

  clear_serial_input(serial_port);
  write(serial_port, buf, 7);
}

int LobotSerialServoReadPosition(int serial_port, uint8_t id)
{
 
  byte buf[6];

  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_POS_READ;
  buf[5] = LobotCheckSum(buf);



  clear_serial_input(serial_port);
  write(serial_port, buf, 6);
  char in_buf[8];
  auto ok = read_with_timeout(serial_port, in_buf, 8);

  if(!ok) cout << "failed to read servo position";
  return (int16_t) BYTE_TO_HW(in_buf[6], in_buf[5]);
}


void config_serial_port(int serial_port) {
  // Create new termios struc, we call it 'tty' for convention
  struct termios tty;

  // Read in existing settings, and handle any error
  if(tcgetattr(serial_port, &tty) != 0) {
    fail("error from tcgetattr");
  }

  tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
  tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
  tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
  tty.c_cflag |= CS8; // 8 bits per byte (most common)
  tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO; // Disable echo
  tty.c_lflag &= ~ECHOE; // Disable erasure
  tty.c_lflag &= ~ECHONL; // Disable new-line echo
  tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
  // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
  // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

  tty.c_cc[VTIME] = 0;    // No blocking, return immediately with what is available
  tty.c_cc[VMIN] = 0;

  // Set in/out baud rate to be 115200
  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);

  // Save tty settings, also checking for error
  if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
      fail("Error from tcsetattr: ");
  }
}

void move_smoothly(int serial_port, int servo_id, int end_pos, float seconds)
{
  auto start_pos = LobotSerialServoReadPosition(serial_port, servo_id);
  auto t1 = std::chrono::high_resolution_clock::now();
  while (true) {
    auto t2 = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration<double>(t2 - t1).count();
    if(elapsed>=seconds) {
      LobotSerialServoMoveTimeWrite(serial_port, servo_id, end_pos, 1);
      break;
    }
    float p = (-cos(elapsed/seconds * M_PI)+1)/2;
    auto pos = start_pos + p*(end_pos-start_pos);
    LobotSerialServoMoveTimeWrite(serial_port, servo_id, pos, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

}

void run() {
    // Open the serial port. Change device path as needed (currently set to an standard FTDI USB-UART cable type device)
  int serial_port = open("/dev/ttyUSB1", O_RDWR);
  config_serial_port(serial_port);

  uint16_t duration = 100;
  int servo_id = 1;

  int current_position = -1;
  for(uint16_t position = 0; position <= 800; position+=100) {
    cout << "moving to position: " << position << endl;
    LobotSerialServoMoveTimeWrite(serial_port, servo_id, position, duration);
    for(int i=0; i<100; ++i) {
      std::this_thread::sleep_for (std::chrono::milliseconds(10));
      current_position = LobotSerialServoReadPosition(serial_port, servo_id);
      cout << "current position: " << current_position << endl;
    }
  }

  // smooth
  move_smoothly(serial_port, servo_id, 793, 10);
  move_smoothly(serial_port, servo_id, 43, 10);
  move_smoothly(serial_port, servo_id, 793, 10);
  move_smoothly(serial_port, servo_id, 43, 10);

  move_smoothly(serial_port, servo_id, 793, 3);
  move_smoothly(serial_port, servo_id, 43, 3);
  move_smoothly(serial_port, servo_id, 793, 3);
  move_smoothly(serial_port, servo_id, 43, 3);

  move_smoothly(serial_port, servo_id, 793, 2);
  move_smoothly(serial_port, servo_id, 43, 2);
  move_smoothly(serial_port, servo_id, 793, 2);
  move_smoothly(serial_port, servo_id, 43, 2);


  LobotServoLoadOrUnloadWrite(serial_port, servo_id, 0);
  return;
  cout << "motor unloaded, in theory can move freely" << endl;
  int last_position = current_position;
  while(true) { 
      auto current_position = LobotSerialServoReadPosition(serial_port, servo_id);
      if(current_position != last_position) {
        cout << "manual position: " << current_position << endl;
        last_position = current_position;
      }
  }


  close(serial_port);
}



int main(int argc, char** arv) {
  // std::fstream f;
  // f.open("/dev/ttyUSB0");
//   cout << "available bytes" << f.rdbuf()->in_avail() << endl;
//   // f.open("/home/brian/play.txt");
//   f.get();
//   return 0;
// }


  try {
    run();
  }
  catch(string s) {
    cout << "Failed, error caught in main: " << s << endl;
  }
  return 0;
}
