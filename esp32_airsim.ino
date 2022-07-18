/* esp32_airsim
 *  Devloped by Sergio Reyes Sanchez
 *  Useful connection using serial port and MAvlink2
 *  To use Airsim as physics simulator and test code
 *  based on the mav connection of Juan Pedro López
 *  
 */

#define SOFT_SERIAL_DEBUGGING   // Comment this line if no serial debugging is needed
#ifdef SOFT_SERIAL_DEBUGGING
  #include <HardwareSerial.h>
  HardwareSerial mySerial(2); // serial connection on esp32
#endif

#include "common/mavlink.h"

// variables
unsigned long previousMillisMAVLink = 0;
unsigned long next_interval_MAVLink = 1000;

float throttle = 0.0f;
float pitch = 0.0f;

float alt = 0;
float vel_z = 0;

bool active_ctrl_z = false;
float ctrl_z = 0;
float home_alt = 0;


void setup() {
  Serial.begin(921600);

#ifdef SOFT_SERIAL_DEBUGGING
  Serial.begin(921600);
  Serial.println("MAVLink starting.");
  #define MYPORT_RX 16
  #define MYPORT_TX 17
  mySerial.begin(230400, SERIAL_8N1, MYPORT_RX, MYPORT_TX);
#endif
}

void loop() {
  unsigned long currentMillis = millis();
  int i=0;
  
  int sysid = 1;                   // normal sysid
  int compid = 191;                // OBC
  int type = MAV_TYPE_QUADROTOR;
 
  uint8_t system_type = MAV_TYPE_GENERIC;
  uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;
  uint8_t system_mode = MAV_MODE_PREFLIGHT;
  uint32_t custom_mode = 0;
  uint8_t system_state = MAV_STATE_STANDBY;

  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
 
  mavlink_msg_heartbeat_pack(1,0, &msg, type, autopilot_type, system_mode, custom_mode, system_state);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  mavlink_message_t msg2;
  mavlink_hil_actuator_controls_t actuators;
  actuators.time_usec = micros();

  if (micros() <12000000) {
    home_alt = alt;
    }
  if (micros() > 14000000 && micros() < 18000000){
    throttle = 0.5;
    active_ctrl_z = true;
    
  }

  float ref_alt;
  if (active_ctrl_z){
    ref_alt = (3000 - (alt - home_alt)) / 10;
    ctrl_z = (ref_alt - vel_z)*0.05; // altitude is in mm
    if (ctrl_z > 0.3){
      ctrl_z = 0.3;
    }else if (ctrl_z < -0.3){
      ctrl_z = -0.3;
    }
  }

  actuators.controls[0]  = throttle + ctrl_z + pitch;
  actuators.controls[1]  = throttle + ctrl_z - pitch;
  actuators.controls[2]  = throttle + ctrl_z + pitch;
  actuators.controls[3]  = throttle + ctrl_z - pitch;
  actuators.controls[4]  = -0.2f;
  actuators.controls[5]  = -0.2f;
  actuators.controls[6]  = -0.2f;
  actuators.controls[7]  = -0.2f;
  actuators.controls[8]  = -0.2f;
  actuators.controls[9]  = -0.2f;
  actuators.controls[10] = -0.2f;
  actuators.controls[11] = -0.2f;
  actuators.controls[12] = -0.2f;
  actuators.controls[13] = -0.2f;
  actuators.controls[14] = -0.2f;
  actuators.controls[15] = -0.2f;
  actuators.mode = MAV_MODE_FLAG_SAFETY_ARMED + MAV_MODE_FLAG_HIL_ENABLED + MAV_MODE_FLAG_STABILIZE_ENABLED + MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;

  mavlink_msg_hil_actuator_controls_encode(sysid, compid, &msg2, &actuators);
  
  uint8_t buf2[MAVLINK_MAX_PACKET_LEN];
  uint16_t len2 = mavlink_msg_to_send_buffer(buf2, &msg2);
  unsigned long currentMillisMAVLink = millis();
  Serial.write(buf2, len2);
  
  if (currentMillisMAVLink - previousMillisMAVLink >= next_interval_MAVLink) {
    previousMillisMAVLink = currentMillisMAVLink;
    Serial.write(buf, len);
  }

  // Check reception buffer
  comm_receive();
}


void comm_receive() {
 
  mavlink_message_t msg;
  mavlink_status_t status;
 

  while(Serial.available()>0) {
    uint8_t c = Serial.read();
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      // Handle message
      switch(msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:  // #0: Heartbeat
          {
#ifdef SOFT_SERIAL_DEBUGGING
            mySerial.println("PX HB");
#endif
          }
          break;

        

          case MAVLINK_MSG_ID_HIL_STATE_QUATERNION:  // #115
          {
            mavlink_hil_state_quaternion_t state_quat;
            mavlink_msg_hil_state_quaternion_decode(&msg, &state_quat);
            alt = state_quat.alt;
#ifdef SOFT_SERIAL_DEBUGGING
            mySerial.println("HILQUAT:");
            mySerial.print("lat ");
            mySerial.println(state_quat.lat);
            mySerial.print("lon ");
            mySerial.println(state_quat.lon);
            mySerial.print("alt ");
            mySerial.println(state_quat.alt);
            mySerial.print("rollspeed ");
            mySerial.println(state_quat.rollspeed);
            mySerial.print("pitchspeed ");
            mySerial.println(state_quat.pitchspeed);
#endif
          }
          break;

          case MAVLINK_MSG_ID_HIL_GPS:  // #113
          {
            mavlink_hil_gps_t hil_gps;
            mavlink_msg_hil_gps_decode(&msg, &hil_gps);
            alt = hil_gps.alt;
            vel_z = - hil_gps.vd;
#ifdef SOFT_SERIAL_DEBUGGING
            mySerial.println("hil_gps:");
            mySerial.print("lat ");
            mySerial.println(hil_gps.lat);
            mySerial.print("lon ");
            mySerial.println(hil_gps.lon);
            mySerial.print("alt ");
            mySerial.println(alt);
            mySerial.print("vel north ");
            mySerial.println(hil_gps.vn);
            mySerial.print("vel east ");
            mySerial.println(hil_gps.ve);
#endif
          }
          break;

          case MAVLINK_MSG_ID_HIL_SENSOR:  // #113
          {
            mavlink_hil_sensor_t hil_sensor;
            mavlink_msg_hil_sensor_decode(&msg, &hil_sensor);
#ifdef SOFT_SERIAL_DEBUGGING
            mySerial.println("hil_sensor:");
            mySerial.print("xacc ");
            mySerial.println(hil_sensor.xacc);
            mySerial.print("yacc ");
            mySerial.println(hil_sensor.yacc);
            mySerial.print("x_gyro ");
            mySerial.println(hil_sensor.xgyro);
            mySerial.print("y_gyro ");
            mySerial.println(hil_sensor.ygyro);
            
#endif
          }
          break;

       default:
#ifdef SOFT_SERIAL_DEBUGGING
          mySerial.print("--- Otros: ");
          mySerial.print("[ID: ");
          mySerial.print(msg.msgid);
          mySerial.print("], [seq: ");
          mySerial.print(msg.seq);
          mySerial.print("]");
          mySerial.print(" VZ: ");
          mySerial.print(vel_z);
          mySerial.print(" CZ: ");
          mySerial.print(ctrl_z);
          mySerial.print(" alt ");
          mySerial.println(alt - home_alt);
#endif
          break;
      }
    }
  }
}
