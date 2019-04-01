#include "MavLink\common\mavlink.h"
#include "mavGlobals.h"
#include <MPU9250.h>
#include <Wire.h>
#include <SPI.h>
#include "TypeDef.h"
#include <Arduino.h>
#include <U8g2lib.h>
#define i2cFreq 1800000
#define rad2deg 180.0/3.1415926536              // convert radian to degree
#define deg2rad 3.1415926536/180.0
#define g 9.8

elapsedMillis bootTime;

/* Create an internal interrupt to do the AHRS Routine */
#define period_micro_sec 2500  // Refer to "TypeDef.h for T again!"

U8G2_SSD1306_128X64_VCOMH0_F_HW_I2C u8g2(U8G2_R0, /* clock=*/ 19, /* data=*/ 18);   // ESP32 Thing, HW I2C with pin remapping

// Teensy Timer Library
IntervalTimer myTimer;

/* AHRS Algorithm typedef structure */
IMU_VAR IMU;
IMU_BIAS IMU_bias;

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU_9250(Wire, 0x68);
int status;

static double Time_Cost = 0;
unsigned long int timer = 0;
int counter = 0;
unsigned short int digit = 4;
double Ax, Ay, Az, Wx, Wy, Wz, Mx, My, Mz, Bz;
double TP;


void Get_Sensor_Data(IMU_VAR *pIMU);
void send_ahrs_msg();
void send_imu_msg();
void send_heartbeat();
void send_status();
void handle_Messages();




void setup()
{
  /* Arduino initializations */
  Wire.begin();
  Serial.begin(115200);
  Wire.setClock(i2cFreq);
  // initialze LED module
  u8g2.begin();
  u8g2.enableUTF8Print();    // enable UTF8 support for the Arduino print() function
  u8g2.clearBuffer();
  u8g2.setFontDirection(0);
  u8g2.setFont(u8g2_font_courR10_tr);
  u8g2.setCursor(5, 14);
  u8g2.print("MPU9255 Reader");
  u8g2.setCursor(15, 45);
  u8g2.sendBuffer();
  delay(100);

  // start communication with IMU
  status = IMU_9250.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while (1) {}
  }
  // setting the accelerometer full scale range to +/-8G
  IMU_9250.setAccelRange(MPU9250::ACCEL_RANGE_8G);
  // setting the gyroscope full scale range to +/-2000 deg/s
  IMU_9250.setGyroRange(MPU9250::GYRO_RANGE_2000DPS);
  // setting DLPF bandwidth to 184 Hz
  IMU_9250.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_184HZ);
  // setting SRD to 1 for a 500 Hz update rate
  IMU_9250.setSrd(0);

  /* Timer Interrupt for AHRS Routine */
  myTimer.begin(Routine_AHRS, period_micro_sec);
  myTimer.priority(0);
}


double int_gyro = 0.0;

void loop()
{
  
  /* Get Sensor Data from I2C */
  IMU_9250.readSensor();
  Time_Cost = double(micros() - timer) / 1e06;
  timer = micros();
  serial_Print();
  
  // display SSD1306 
  if (counter % 10 == 0)
  {
    miniTFT_Print();
    counter = 0;
  }
  counter += 1;

  //  int_gyro += IMU.Gyro[0]*Time_Cost;





  /*
   *******************
   *  Begin MAVLINK  *
   *******************
   */
   
  //Pack and send heartbeat at specific interval to the GCS
  if((micros() - heartbeatTimer_TX) > heartbeatInterval_TX)
  {
    heartbeatTimer_TX = micros();
    send_heartbeat();
    send_status();  //fake data sent for now.  Seen this at .5sec
  }
  
  if((micros() - ahrsTimer) > ahrsInterval)
  {
    send_ahrs_msg(); 
    send_imu_msg(); 
    ahrsTimer = micros();
  }

  /* Cuz no GPS data
  // Pack and send GPS reading at the set interval
  if((micros() - gpsTimer) > gpsInterval)
  { 
    send_gps_msg();
        gpsTimer = micros(); // reset the timer
  }
  */

   handle_Messages();

}



void getIMU()
{
  Ax = (double)   IMU_9250.getAccelY_mss(); // m/sec
  Ay = (double)   IMU_9250.getAccelX_mss(); // m/sec
  Az = (double)-1*IMU_9250.getAccelZ_mss(); // m/sec
  Wx = (double)   IMU_9250.getGyroY_rads(); // rad/sec
  Wy = (double)   IMU_9250.getGyroX_rads(); // rad/sec
  Wz = (double)-1*IMU_9250.getGyroZ_rads(); // rad/sec
  Mx = (double)   IMU_9250.getMagY_uT(); // uT
  My = (double)   IMU_9250.getMagX_uT(); // uT
  Mz = (double)-1*IMU_9250.getMagZ_uT(); // uT
  TP = (double)   IMU_9250.getTemperature_C();  // degreeC
}


/* AHRS Routine called by internal interrupt */
void Routine_AHRS()
{
  /* Convert I2C Data to Physical Quantity */
  getIMU();
  Get_Sensor_Data(&IMU);
  
}


void serial_Print()
{
  Serial.flush();
  Serial.print(IMU.Acc[0], digit);
  Serial.print("   ");
  Serial.print(IMU.Acc[1], digit);
  Serial.print("   ");
  Serial.print(IMU.Acc[2], digit);
  Serial.print("   ");
  Serial.print(IMU.Gyro[0] * rad2deg, digit);
  Serial.print("   ");
  Serial.print(IMU.Gyro[1] * rad2deg, digit);
  Serial.print("   ");
  Serial.print(IMU.Gyro[2] * rad2deg, digit);
  Serial.print("   ");
  Serial.print(IMU.Mag[0], digit);
  Serial.print("   ");
  Serial.print(IMU.Mag[1], digit);
  Serial.print("   ");
  Serial.print(IMU.Mag[2], digit);
  Serial.print("   ");
  Serial.print(IMU.TP, digit);
  Serial.print("   ");  
  Serial.println(Time_Cost, 4);
}



void miniTFT_Print()
{
  u8g2.clearBuffer();
  u8g2.setFontDirection(0);
  u8g2.setFont(u8g2_font_courR08_tr);
  u8g2.setCursor(0, 6);
  u8g2.print("IEC-Lab IMU Display");
  u8g2.setFont(u8g2_font_helvB08_tf);
  u8g2.setCursor(0, 24);
  u8g2.print("Wx = ");
  u8g2.print(IMU.Gyro[0] * rad2deg, 6);
  u8g2.print("°");
  u8g2.setCursor(0, 38);
  u8g2.print("Wy = ");
  u8g2.print(IMU.Gyro[1] * rad2deg, 6);
  u8g2.print("°");
  u8g2.setCursor(0, 50);
  u8g2.print("Wz = ");
  u8g2.print(IMU.Gyro[2] * rad2deg, 6);
  u8g2.print("°");
  u8g2.setCursor(0, 64);
  u8g2.print("TP = ");
  u8g2.print(IMU.TP, 6);
  u8g2.print("°");  
  u8g2.sendBuffer();
}


/**************************************************************************************************************
  Function Name     :   Get_Sensor_Data(IMU_VAR *pIMU)
  Function Input    :       (1). IMU_VAR structure variable
  Function output   :       (1). IMU_VAR structure variable
  Creator           :    Dr. Chao-Chung Peng, Stud. J.J. Huang
  Create Date       :   2017/ 3 / 3
  Reference         :   NA
  Describe          :   The function is used to get sensor data
  History           :   NA
  Return value      :   VOID
***************************************************************************************************************/
void Get_Sensor_Data(IMU_VAR *pIMU)
{
  pIMU->Acc[0]  = Ax;
  pIMU->Acc[1]  = Ay;
  pIMU->Acc[2]  = Az;
  pIMU->Gyro[0] = Wx;
  pIMU->Gyro[1] = Wy;
  pIMU->Gyro[2] = Wz;
  pIMU->Mag[0]  = Mx;
  pIMU->Mag[1]  = My;
  pIMU->Mag[2]  = Mz;
  pIMU->TP  = TP;
}

void send_heartbeat(){
    memset(bufTx, 0xFF, sizeof(bufTx));
    mavlink_system.sysid = MAV_TYPE_GROUND_ROVER ;
    mavlink_system.compid = MAV_COMP_ID_AUTOPILOT1; 
    
    // Pack the message 
    mavlink_msg_heartbeat_pack(mavlink_system.sysid, mavlink_system.compid, &heartbeatMsg, system_type, autopilot_type, heartbeat.base_mode, heartbeat.custom_mode, heartbeat.system_status);
  
    // Copy the message to send buffer 
    uint16_t len = mavlink_msg_to_send_buffer(bufTx, &heartbeatMsg);
 
    //Write Message    
    Serial.write(bufTx, len);        
    heartbeatTimer_TX = millis();
    Serial.println("Heartbeat"); 
}


void send_ahrs_msg(){
    memset(bufTx, 0xFF, sizeof(bufTx));
    mavlink_msg_attitude_pack(mavlink_system.sysid, mavlink_system.compid, &ahrsMsg,
                              bootTime,
                              IMU.Roll,
                              IMU.Pitch,
                              IMU.Yaw,
                              IMU.Gyro[0] * rad2deg,
                              IMU.Gyro[1] * rad2deg,
                              IMU.Gyro[2] * rad2deg);
    
    /// Copy the message to send buffer
    uint16_t len = mavlink_msg_to_send_buffer(bufTx, &ahrsMsg);

     //Write Message    
    Serial.write(bufTx, len);  
}

void send_imu_msg(){
    memset(bufTx, 0xFF, sizeof(bufTx));
    mavlink_msg_raw_imu_pack(mavlink_system.sysid, mavlink_system.compid, &imuMsg,
                              bootTime,
                              IMU.Acc[0],
                              IMU.Acc[1],
                              IMU.Acc[2],
                              IMU.Gyro[0] * rad2deg,
                              IMU.Gyro[1] * rad2deg,
                              IMU.Gyro[2] * rad2deg,
                              IMU.Mag[0],
                              IMU.Mag[1],
                              IMU.Mag[2]);
    
    /// Copy the message to send buffer
    uint16_t len = mavlink_msg_to_send_buffer(bufTx, &imuMsg);

     //Write Message    
    Serial.write(bufTx, len);  
}

void send_status(){
    memset(bufTx, 0xFF, sizeof(bufTx));
    mavlink_msg_sys_status_pack(mavlink_system.sysid, mavlink_system.compid, &statMsg,
      MAVLINK_SENSOR_PRESENT_DEFAULT, MAVLINK_SENSOR_PRESENT_DEFAULT, 
      MAVLINK_SENSOR_PRESENT_DEFAULT, 500,7400,330,50,0,0,0,0,0,0);
    
    /// Copy the message to send buffer
    uint16_t len = mavlink_msg_to_send_buffer(bufTx, &statMsg);

     //Write Message    
    Serial.write(bufTx, len);  
}

void handle_Messages(){
  while(Serial.available()>0) {
    uint8_t c = Serial.read();
    /*
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &receivedMsg, &mav_status)) {
        //print_heartbeat();
        if(receivedMsg.msgid > 0){
          //Serial.print(" -> Msg ID: ");
          //Serial.println(receivedMsg.msgid, DEC);
          if(receivedMsg.msgid == 11){
            if(mode.base_mode == MAV_MODE_GUIDED_ARMED || mode.base_mode == 
              MAV_MODE_MANUAL_ARMED){
                //Serial.println("SYSTEM ARMED");
                //print_setMode();
                //motor_control = 1;
              } else {
                Serial.print("--> New Base Mode: ");
                Serial.println(mode.base_mode);
              }
          }
        }
        */
        switch(receivedMsg.msgid)
        {
          //Serial.println(receivedMsg.msgid);
          case MAVLINK_MSG_ID_SYS_STATUS:  // #1: SYS_STATUS
            {
              /* Message decoding: PRIMITIVE
               *    mavlink_msg_sys_status_decode(const mavlink_message_t* msg, mavlink_sys_status_t* sys_status)
               */
              //mavlink_message_t* msg;
              mavlink_sys_status_t sys_status;
              mavlink_msg_sys_status_decode(&msg, &sys_status);
            }
            break;
          /*
          case MAVLINK_MSG_ID_MANUAL_CONTROL: // #69: Joystick data
                //[IMPORTANT: TEMPORARY FIX EXPLAINED BELOW]
                mavlink_msg_manual_control_decode(&receivedMsg, &manual_control);
                //Serial.println("Received Manual Message");
                joyStick_control();
                break;
                
          case MAVLINK_MSG_ID_SET_MODE:            //Get new base mode
              //killMotors();                        //Get bot ready for new mode
              mavlink_msg_set_mode_decode(&receivedMsg, &mode);
              heartbeat.base_mode = mode.base_mode;
              heartbeat.custom_mode = mode.custom_mode;
              print_setMode();
              break;
          */
          case MAVLINK_MSG_ID_HEARTBEAT: //#0 Get new heartbeat 
                 //[IMPORTANT: Mavlink C++ generator decodes the heartbeat incorrectly (parameters out of order)]
                 heartbeatTimer_RX = micros(); //Reset receive timer 
                 //print_newHearbeat();
                 break;
           /*
           case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: //#21  
               send_parameters();
               break;

           case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
               send_parameters();
               break;

           case MAVLINK_MSG_ID_MISSION_ITEM:
                mavlink_msg_mission_item_decode(&receivedMsg, &misItem);
                print_mission_item();
                save_waypoints();
                send_mission_ack();
                break;

           case MAVLINK_MSG_ID_MISSION_COUNT:
               mavlink_msg_mission_count_decode(&receivedMsg, &misCount);
               wp_count = misCount.count;
               if(wp_count > 100 || wp_count == 0) {
                  wp_count = 0;
                  break;
               } else {
                 send_mission_req();
               }
               break;

          case MAVLINK_MSG_ID_PARAM_SET:  
             //Serial.println("GOT PARAM_SET");
             mavlink_param_set_t packet;
             mavlink_msg_param_set_decode(&paramMsg, &packet);
             //Serial.print("Received set parameter: "); 
             //Serial.print(packet.param_id[0]); Serial.print(",  ");
             //Serial.println(packet.param_value);
             send_mission_ack();
             
             break;

          case MAVLINK_MSG_ID_PARAM_VALUE:
             Serial.println("GOT PARAM_VALUE");
             mavlink_param_set_t packet1;
             mavlink_msg_param_set_decode(&paramMsg, &packet1);
             Serial.print("Received set parameter: ");
             Serial.print(packet1.param_id[0],HEX); Serial.print(",  ");
             Serial.println(packet1.param_value);
             break;
          */
          
          // case MAVLINK_MSG_ID_COMMAND_LONG:
            // mavlink_msg_command_long_decode(&receivedMsg, &cmd_long);
            // Serial.print("\tTarget Component: "); Serial.println(cmd_long.target_component);
            // Serial.print("\tCommand: "); Serial.println(cmd_long.command);
            // Serial.print("\tParam1: "); Serial.println(cmd_long.param1);
            // Serial.print("\tParam2: "); Serial.println(cmd_long.param2);
            // Serial.print("\tParam3: "); Serial.println(cmd_long.param3);
            // Serial.print("\tParam4: "); Serial.println(cmd_long.param4);
            // Serial.print("\tParam5: "); Serial.println(cmd_long.param5);
            // Serial.print("\tParam6: "); Serial.println(cmd_long.param6);
            // Serial.print("\tParam7: "); Serial.println(cmd_long.param7);           
            // send_mission_ack();
          // break;

        }
      
    }

}