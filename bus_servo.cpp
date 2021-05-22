#include <iostream>
#include <fstream>
#include <iomanip>

#include <string>
#include <thread>
#include <chrono>

#include "string.h"

#include <stdio.h>
#include <string.h>
#include <cmath>
#include <cassert>


#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()


using namespace std;

typedef uint8_t byte;

#define GET_LOW_BYTE(A) (uint8_t)((A))
//Macro function  get lower 8 bits of A
#define GET_HIGH_BYTE(A) (uint8_t)((A) >> 8)
//Macro function  get higher 8 bits of A
#define BYTE_TO_HW(A, B) ((((uint16_t)(A)) << 8) | (uint8_t)(B))

struct ServoCommand {
  ServoCommand(char id, int length) : id(id), length(length) {}
  const char id;
  const int length;
};

// servo read commands have one more field than servo commands
struct ServoReadCommand : public ServoCommand {
  ServoReadCommand(char id, int length, int read_length) : 
    ServoCommand(id, length), read_length(read_length) 
    {
      assert(length==3); // read commands never send parameters
    }
  const int read_length;
};

#define SERVO_FRAME_HEADER         0x55

// sending to this servo_id will be broadcast to all connected servos
#define BROADCAST_ID 0xFE

// see Table1 and Table4 of "LewanSoul Bus Servo Communication Protocol.pdf"
//
const ServoCommand     SERVO_MOVE_TIME_WRITE      (1, 7);
const ServoReadCommand SERVO_MOVE_TIME_READ       (2, 3, 7);
const ServoCommand     SERVO_MOVE_TIME_WAIT_WRITE (7, 7);
const ServoReadCommand SERVO_MOVE_TIME_WAIT_READ  (8, 3, 7);
const ServoCommand     SERVO_MOVE_START           (11, 3);
const ServoCommand     SERVO_MOVE_STOP            (12, 3);
const ServoCommand     SERVO_ID_WRITE             (13, 4);
const ServoReadCommand SERVO_ID_READ              (14, 3, 4);
const ServoCommand     SERVO_ANGLE_OFFSET_ADJUST  (17, 4);
const ServoCommand     SERVO_ANGLE_OFFSET_WRITE   (18, 3);
const ServoReadCommand SERVO_ANGLE_OFFSET_READ    (19, 3, 4);
const ServoCommand     SERVO_ANGLE_LIMIT_WRITE    (20, 7);
const ServoReadCommand SERVO_ANGLE_LIMIT_READ     (21, 3, 7);
const ServoCommand     SERVO_VIN_LIMIT_WRITE      (22, 7);
const ServoReadCommand SERVO_VIN_LIMIT_READ       (23, 3, 7);
const ServoCommand     SERVO_TEMP_MAX_LIMIT_WRITE (24, 4);
const ServoReadCommand SERVO_TEMP_MAX_LIMIT_READ  (25, 3, 4);
const ServoReadCommand SERVO_TEMP_READ            (26, 3, 4);
const ServoReadCommand SERVO_VIN_READ             (27, 3, 5);
const ServoReadCommand SERVO_POS_READ             (28, 3, 5);
const ServoCommand     SERVO_OR_MOTOR_MODE_WRITE  (29, 7);
const ServoReadCommand SERVO_OR_MOTOR_MODE_READ   (30, 3, 7);
const ServoCommand     SERVO_LOAD_OR_UNLOAD_WRITE (31, 4);
const ServoReadCommand SERVO_LOAD_OR_UNLOAD_READ  (32, 3, 4);
const ServoCommand     SERVO_LED_CTRL_WRITE       (33, 4);
const ServoReadCommand SERVO_LED_CTRL_READ        (34, 3, 4);
const ServoCommand     SERVO_LED_ERROR_WRITE      (35, 4);
const ServoReadCommand SERVO_LED_ERROR_READ       (36, 3, 4);

byte check_sum(byte buf[])
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

#include <termios.h> // for tcflush
void clear_serial_input(int serial_port) {
  tcflush(serial_port, TCIFLUSH); // fastest way to clear serial buffer
}


// send_command has versions for each parameter lenght

void write_command_0(ServoCommand cmd, int serial_port, uint8_t servo_id) {
  assert(cmd.length==3);
  const int buf_length = 6;
  byte buf[buf_length];
  buf[0] = buf[1] = SERVO_FRAME_HEADER;
  buf[2] = servo_id;
  buf[3] = cmd.length;
  buf[4] = cmd.id;
  buf[5] = check_sum(buf);

  write(serial_port, buf, buf_length);
}

void write_command_1(ServoCommand cmd, int serial_port, uint8_t servo_id, byte parameter1) {
  assert(cmd.length==4);
  const int buf_length = 7;
  byte buf[buf_length];
  buf[0] = buf[1] = SERVO_FRAME_HEADER;
  buf[2] = servo_id;
  buf[3] = cmd.length;
  buf[4] = cmd.id;
  buf[5] = parameter1;
  buf[6] = check_sum(buf);

  write(serial_port, buf, buf_length);
}


void write_command_4(ServoCommand cmd, int serial_port, uint8_t servo_id, byte parameter1, byte parameter2, byte parameter3, byte parameter4) {
  assert(cmd.length==7);
  const int buf_length = 10;
  byte buf[buf_length];
  buf[0] = buf[1] = SERVO_FRAME_HEADER;
  buf[2] = servo_id;
  buf[3] = cmd.length;
  buf[4] = cmd.id;
  buf[5] = parameter1;
  buf[6] = parameter2;
  buf[7] = parameter3;
  buf[8] = parameter4;
  buf[9] = check_sum(buf);

  write(serial_port, buf, buf_length);
}

// returns true on success
bool read_with_timeout(int serial_port, byte * buf, int len, int timeout_ms = 50) {
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
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }
}

bool read_command_1(ServoReadCommand cmd, int serial_port, uint8_t servo_id, byte *param1) {
  clear_serial_input(serial_port);
  write_command_0(cmd, serial_port, servo_id);
  const int buf_length = 7;
  byte buf[buf_length];
  auto ok = read_with_timeout(serial_port, buf, buf_length);
  if(!ok)
    return false;
  *param1 = buf[5];
  return true;
}

bool read_command_2(ServoReadCommand cmd, int serial_port, uint8_t servo_id, byte *param1, byte * param2) {
  clear_serial_input(serial_port);
  write_command_0(cmd, serial_port, servo_id);
  const int buf_length = 8;
  byte buf[buf_length];
  auto ok = read_with_timeout(serial_port, buf, buf_length);
  if(!ok)
    return false;
  *param1 = buf[5];
  *param2 = buf[6];
  return true;
}

bool read_command_4(ServoReadCommand cmd, int serial_port, uint8_t servo_id, byte *param1, byte *param2, byte* param3, byte* param4) {
  clear_serial_input(serial_port);
  write_command_0(cmd, serial_port, servo_id);
  const int buf_length = 10;
  byte buf[buf_length];
  auto ok = read_with_timeout(serial_port, buf, buf_length);
  if(!ok)
    return false;
  *param1 = buf[5];
  *param2 = buf[6];
  *param3 = buf[7];
  *param4 = buf[8];
  return true;
}

void servo_move_time_write(int serial_port, uint8_t servo_id, uint16_t position, uint16_t ms)
{
  byte param1 = GET_LOW_BYTE(position);  // lower 8 bits of angle
  byte param2 = GET_HIGH_BYTE(position); // higher 8 bits of angle
  byte param3 = GET_LOW_BYTE(ms);        // lower 8 bits of time in ms (0-3000)
  byte param4 = GET_HIGH_BYTE(ms);       // higher 8 bits of time 

  write_command_4(SERVO_MOVE_TIME_WRITE, serial_port, servo_id, param1, param2, param3, param4);
}

// reads last values sent to server_move_time_write
bool servo_move_time_read(int serial_port, uint8_t servo_id, int16_t * position, uint16_t * ms) {
  /*
  Parameter 1: lower 8 bits of angle value
  Parameter 2: higher 8 bits of angle, range 0~1000
  Parameter 3: lower 8 bits of time value
  Parameter 4: higher 8 bits of time value, range 0~30000ms
  */
  const auto cmd = SERVO_MOVE_TIME_READ;
  byte param1, param2, param3, param4;
  bool ok = read_command_4(cmd, serial_port, servo_id, &param1, &param2, &param3, &param4);
  if(!ok) return false;

  *position = (int16_t) BYTE_TO_HW(param2, param1);
  *ms = BYTE_TO_HW(param4, param3);
  return true;
}

void servo_move_time_wait_write(int serial_port, uint8_t servo_id, uint16_t position, uint16_t ms)
{
  byte param1 = GET_LOW_BYTE(position);  // lower 8 bits of angle
  byte param2 = GET_HIGH_BYTE(position); // higher 8 bits of angle
  byte param3 = GET_LOW_BYTE(ms);        // lower 8 bits of time in ms (0-3000)
  byte param4 = GET_HIGH_BYTE(ms);       // higher 8 bits of time 

  write_command_4(SERVO_MOVE_TIME_WAIT_WRITE, serial_port, servo_id, param1, param2, param3, param4);
}

// reads last values sent to server_move_time_wait_write
bool servo_move_time_wait_read(int serial_port, uint8_t servo_id, int16_t * position, uint16_t * ms) {
  /*
  Parameter 1: lower 8 bits of angle value
  Parameter 2: higher 8 bits of angle, range 0~1000
  Parameter 3: lower 8 bits of time value
  Parameter 4: higher 8 bits of time value, range 0~30000ms
  */
  const auto cmd = SERVO_MOVE_TIME_WAIT_READ;
  byte param1, param2, param3, param4;
  bool ok = read_command_4(cmd, serial_port, servo_id, &param1, &param2, &param3, &param4);
  if(!ok) return false;

  *position = (int16_t) BYTE_TO_HW(param2, param1);
  *ms = BYTE_TO_HW(param4, param3);
  return true;
}

void servo_move_start(int serial_port, uint8_t servo_id) {
  write_command_0(SERVO_MOVE_START, serial_port, servo_id);
}

void servo_move_stop(int serial_port, uint8_t servo_id) {
  write_command_0(SERVO_MOVE_STOP, serial_port, servo_id);
}

// permanently changes servo at id servo_id to new_servo_id
void servo_id_write(int serial_port, uint8_t servo_id, uint8_t new_servo_id) {
  write_command_1(SERVO_ID_WRITE, serial_port, servo_id, new_servo_id);
}


bool servo_id_read(int serial_port, uint8_t servo_id, uint8_t * returned_servo_id) {
  return read_command_1(SERVO_ID_READ, serial_port, servo_id, returned_servo_id);
}

// servo deviation, range -125~ 125, The corresponding angle of
// -30 ° ~ 30 °, when this command reach to the servo, the servo will immediately
// rotate to adjust the deviation
// 
// The adjusted deviation value is not saved when power-down by this
// command, if you want to save, call servo_angle_offset_write()
void servo_angle_offset_adjust(int serial_port, uint8_t servo_id, int8_t offset) {
  write_command_1(SERVO_ANGLE_OFFSET_ADJUST, serial_port, servo_id, (uint8_t) offset);
}

// saves value written from servo_angle_offset_adjust to non-volatile memory
void servo_angle_offset_write(int serial_port, uint8_t servo_id) {
  write_command_0(SERVO_ANGLE_OFFSET_WRITE, serial_port, servo_id);
}

bool servo_angle_offset_read(int serial_port, uint8_t servo_id, int8_t * offset) {
  return read_command_1(SERVO_ANGLE_OFFSET_READ, serial_port, servo_id, (byte *)offset);
}

// min and max positio must be in range 0-1000
// min must be less that max
// saves on power down
void servo_angle_limit_write(int serial_port, uint8_t servo_id, uint16_t min_position, uint16_t max_position) {
  byte param1 = GET_LOW_BYTE(min_position);
  byte param2 = GET_HIGH_BYTE(min_position);
  byte param3 = GET_LOW_BYTE(max_position);
  byte param4 = GET_HIGH_BYTE(max_position);  

  write_command_4(SERVO_ANGLE_LIMIT_WRITE, serial_port, servo_id, param1, param2, param3, param4);
}

bool servo_angle_limit_read(int serial_port, uint8_t servo_id, uint16_t * min_position, uint16_t * max_position) {
  byte param1, param2, param3, param4;
  bool ok =  read_command_4(SERVO_ANGLE_LIMIT_READ, serial_port, servo_id, & param1, & param2, & param3, & param4);
  if(!ok) return false;
  *min_position = BYTE_TO_HW(param2, param1);
  *max_position = BYTE_TO_HW(param4, param3);
  return true;
}


void servo_vin_limit_write(int serial_port, uint8_t servo_id, uint16_t min_vin, uint16_t max_vin) {
  byte param1 = GET_LOW_BYTE(min_vin);
  byte param2 = GET_HIGH_BYTE(min_vin);
  byte param3 = GET_LOW_BYTE(max_vin);
  byte param4 = GET_HIGH_BYTE(max_vin);  

  write_command_4(SERVO_VIN_LIMIT_WRITE, serial_port, servo_id, param1, param2, param3, param4);
}

bool servo_vin_limit_read(int serial_port, uint8_t servo_id, uint16_t * min_vin, uint16_t * max_vin) {
  byte param1, param2, param3, param4;
  bool ok =  read_command_4(SERVO_VIN_LIMIT_READ, serial_port, servo_id, & param1, & param2, & param3, & param4);
  if(!ok) return false;
  *min_vin = BYTE_TO_HW(param2, param1);
  *max_vin = BYTE_TO_HW(param4, param3);
  return true;
}


// The maximum temperature limit inside the servo range
// 50~100°C, the default value is 85°C, if the internal temperature of the servo
// exceeds this value the led will flash and alarm (if an LED alarm is set). In order
// to protect the servo, the motor will be in the unloaded power situation, and the
// servo will not output torque until the temperature below this value of the servo,
// then it will once again enter the working state.and this value supports for
// power-down save.
void servo_temp_max_limit_write(int serial_port, uint8_t servo_id, uint8_t temp_max_limit) {
  write_command_1(SERVO_TEMP_MAX_LIMIT_WRITE, serial_port, servo_id, temp_max_limit);
}

bool servo_temp_max_limit_read(int serial_port, uint8_t servo_id, uint8_t * temp_max_limit) {
  return read_command_1(SERVO_TEMP_MAX_LIMIT_READ, serial_port, servo_id, temp_max_limit);
}


// temp in deg c
bool servo_temp_read(int serial_port, uint8_t servo_id, uint8_t * temp) {
  return read_command_1(SERVO_TEMP_READ, serial_port, servo_id, temp);
}

bool servo_vin_read(int serial_port, uint8_t servo_id, uint16_t * vin) {
  byte param1, param2;
  auto ok = read_command_2(SERVO_VIN_READ, serial_port, servo_id, &param1, &param2);
  if(!ok) return false;
  *vin = BYTE_TO_HW(param2, param1);
  return true;

}


bool servo_pos_read(int serial_port, uint8_t servo_id, int16_t * position) {
  byte param1, param2;
  bool ok = read_command_2(SERVO_POS_READ, serial_port, servo_id, & param1, & param2);
  if(!ok) return false;
  *position = (int16_t)BYTE_TO_HW(param2, param1);
  return true;
}

// mode 0 is servo, 1 is motor
// speed is in range -1000,1000 (tbd: units)
void servo_or_motor_mode_write(int serial_port, uint8_t servo_id, byte mode, int16_t speed ) {
  byte param1 = mode;
  byte param2 = 0; // unused
  byte param3 = GET_LOW_BYTE(speed);
  byte param4 = GET_HIGH_BYTE(speed);
  write_command_4(SERVO_OR_MOTOR_MODE_WRITE, serial_port, servo_id, param1, param2, param3, param4);
}

bool servo_or_motor_mode_read(int serial_port, uint8_t servo_id, byte * mode, int16_t * speed ) {
  byte param1, param2, param3, param4;
  bool ok = read_command_4(SERVO_OR_MOTOR_MODE_READ, serial_port, servo_id, &param1, & param2, &param3, &param4);
  if(!ok) return false;
  *mode = param1;
  *speed = (int16_t)BYTE_TO_HW(param4, param3);
  return true;
}




// load 0 is unpowered, 1 is powered
void servo_load_or_unload_write(int serial_port, uint8_t servo_id, uint8_t load){
  write_command_1(SERVO_LOAD_OR_UNLOAD_WRITE, serial_port, servo_id, load);
}

// load 0 is unpowered, 1 is powered
bool servo_load_or_unload_read(int serial_port, uint8_t servo_id, uint8_t * load){
  return read_command_1(SERVO_LOAD_OR_UNLOAD_READ, serial_port, servo_id, load);
}


// LED light/off state, the range 0 or 1, 0 represents that the LED is
// always on. 1 represents the LED off, the default 0, and support power-down
// save
void servo_led_ctrl_write(int serial_port, uint8_t servo_id, uint8_t value) {
  write_command_1(SERVO_LED_CTRL_WRITE, serial_port, servo_id, value);
}

bool servo_led_ctrl_read(int serial_port, uint8_t servo_id, uint8_t * value) {
  return read_command_1(SERVO_LED_CTRL_READ, serial_port, servo_id, value);
}

// There are three types of faults that cause the LED to flash and alarm,
// regardless of whether the LED is in or off. The first fault is that internal
// temperature of the servo exceeds the maximum temperature limit. 
// The second fault is that the servo input voltage exceeds the
// limit value. The third one is
// when locked-rotor occurred.
// 
//  This value corresponds to the fault alarm relationship as shown below:
//
// 0 No alarm
// 1 Over temperature
// 2 Over voltage
// 3 Over temperature and over voltage
// 4 Locked-rotor
// 5 Over temperature and stalled
// 6 Over voltage and stalled
// 7 Over voltag. Over temperature and stalled
void servo_led_error_write(int serial_port, uint8_t servo_id, uint8_t error) {
  write_command_1(SERVO_LED_ERROR_WRITE, serial_port, servo_id, error);
}

bool servo_led_error_read(int serial_port, uint8_t servo_id, uint8_t * error) {
  return read_command_1(SERVO_LED_ERROR_READ, serial_port, servo_id, error);
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

  // tty.c_cc[VTIME] = 0;    // No blocking, return immediately with what is available
  tty.c_cc[VTIME] = 1;    // Tenth's of a second timeout between bytes
  tty.c_cc[VMIN] = 0;   // return after this number of characters, regardless of number requested 0-unlimited

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
  int16_t start_pos;
  bool ok = servo_pos_read(serial_port, servo_id, &start_pos);
  if(!ok) fail("couldn't get start servo position");
  auto t1 = std::chrono::high_resolution_clock::now();
  while (true) {
    auto t2 = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration<double>(t2 - t1).count();
    if(elapsed>=seconds) {
      servo_move_time_write(serial_port, servo_id, end_pos, 10);
      break;
    }
    float p = (-cos(elapsed/seconds * M_PI)+1)/2;
    auto pos = start_pos + p*(end_pos-start_pos);
    servo_move_time_write(serial_port, servo_id, pos, 10);

    uint8_t error, temp;
    int16_t pos_in;
    uint16_t vin;
    static int count = 0;
    ++count;
    bool ok = servo_vin_read(serial_port, servo_id, &vin);
    if (ok) {
      ok = servo_led_error_read(serial_port, servo_id, &error);
    }
    if (ok) {
      ok = servo_temp_read(serial_port, servo_id, &temp);
    }
    if (ok) {
      ok = servo_pos_read(serial_port, servo_id, &pos_in);
    }
    if(ok) {
      if(count%10==0) {
        cout << "\r"<< "count: " << setw(6) << count << " vin: " << setw(4) << (int)vin << " temp: " << setw(4) << (int)temp <<  " pos: " << setw(4) << pos_in << " error: " << setw(4) << (int)error << std::flush; 
      }
    } else {
      cout << "error reading stats" << endl;
    }

    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }

}

void run() {
    // Open the serial port. Change device path as needed (currently set to an standard FTDI USB-UART cable type device)
  int serial_port = open("/dev/ttyUSB0", O_RDWR);
  config_serial_port(serial_port);
  cout << "running" << endl;

  uint16_t duration = 100;
  int servo_id = 1;
  int servo_id2 = 2;

  servo_vin_limit_write(serial_port, servo_id, 7000, 9000);
  //servo_led_error_write(serial_port, servo_id, 0);
  servo_led_ctrl_write(serial_port, servo_id, 1);

/*
  servo_move_time_write(serial_port, servo_id, 500, 0);

  uint16_t current_position = 0;
  for(int16_t position = 0; position <= 800; position+=100) {
    cout << "moving to position: " << position << endl;
    servo_move_time_write(serial_port, servo_id, position, duration);


    for(int i=0; i<100; ++i) {
      std::this_thread::sleep_for (std::chrono::milliseconds(10));
      int16_t current_position;
      bool ok = servo_pos_read(serial_port, servo_id, &current_position);
      if(!ok) fail("coudn't read servo position");
      cout << "current position: " << current_position << endl;

      int16_t position_out;
      uint16_t duration_out;
      if(! servo_move_time_read(serial_port, servo_id, &position_out, &duration_out)) {
        cout << "***** servo_move_time_read FAILED ****** " << endl;
      } else {
        cout << "servo_move_time_read returned position: " << position_out << " duration: " << duration_out << endl;
      }

    }
  }
*/

  // smooth
  move_smoothly(serial_port, servo_id, 793, 4);
  move_smoothly(serial_port, servo_id, 43, 4);
  move_smoothly(serial_port, servo_id2, 793, 3);
  move_smoothly(serial_port, servo_id2, 43, 3);

  servo_load_or_unload_write(serial_port, servo_id, 0);

  cout << "motor unloaded, now it can move freely" << endl;
  int16_t last_position;
  int16_t current_position;
  int count = 0;
  while(true) { 
      bool ok = servo_pos_read(serial_port, servo_id, &current_position);
      if(!ok) fail("couldn't get start servo position");
      ++count;
      if(abs(current_position - last_position) > 1 || (count % 1000 == 0) ) {
        cout << "count: " << count << " position: " << current_position << endl;
        last_position = current_position;
      }
    // std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  close(serial_port);
  return;
}



int main(int argc, char** arv) {
  try {
    run();
  }
  catch(string s) {
    cout << "Failed, error caught in main: " << s << endl;
  }
  return 0;
}
