//                                ###                                                                                         
//                              # ### #                                                                                      
//                             #### ####                                                                                      
//                             #### #### %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
//                              # ### #  %% %% %%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//                                 #   % % %% %%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
//                              %%%%% %% % %% %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%     
//                             %%%%%% % %% %% %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
//                            %%%%%%% % % %% %%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%      
//                           %%%%%%%%                                                                                         
//                          %%%%%%%%%%                        %%%%%%%%%%%%%%%              %%%%%%%%%%%%%%%%%%%%%%%%%%%%       
//                         %%%%%%%%%%%                    %%%%%%%%%%%%%%%%%%%%%%           %%%%%%%%%%%%%%%%%%%%%%%%%%%%       
//                        %%%%%%%%%%%%                  %%%%%%%%%%%%%%%%%%%%%%%%%          %%%%%%%%%%%%%%%%%%%%%%%%%%%%       
//                       %%%%%%%%%%%%%                %%%%%%%%%%%%%%%  %%%%%%%%%%%        %%%%%%%%%%%%%%%%%%%%%%%%%%%%        
//                      %%%%%%%%%%%%%%%             %%%%%%%%%%%%%%%%      %%%%%%%%%       %%%%%%%%%%%%                        
//                     %%%%%%%%%%%%%%%%            %%%%%%%%%%%%%%%%%       %%%%%%%%      %%%%%%%%%%%%                         
//                    %%%%%%%% %%%%%%%%           %%%%%%%%%%%%%%%%%        %%%%%%%%      %%%%%%%%%%%%                         
//                   %%%%%%%%   %%%%%%%          %%%%%%%%%%%%%%%%%%                      %%%%%%%%%%%                          
//                  %%%%%%%%    %%%%%%%%         %%%%%%%%%%%%%%%%%%                     %%%%%%%%%%%%%%%%%%%%%%%%%%%           
//                 %%%%%%%%     %%%%%%%%        %%%%%%%%%%%%%%%%%%                      %%%%%%%%%%%%%%%%%%%%%%%%%%            
//               %%%%%%%%%      %%%%%%%%        %%%%%%%%%%%%%%%%%%                     %%%%%%%%%%%%%%%%%%%%%%%%%%%            
//              %%%%%%%%%       %%%%%%%%       %%%%%%%%%%%%%%%%%%                      %%%%%%%%%%%%%%%%%%%%%%%%%%             
//             %%%%%%%%%         %%%%%%%%      %%%%%%%%%%%%%%%%%%                      %%%%%%%%%%%                            
//            %%%%%%%%%%%%%%%%%%%%%%%%%%%      %%%%%%%%%%%%%%%%%                      %%%%%%%%%%%%                            
//           %%%%%%%%%%%%%%%%%%%%%%%%%%%%      %%%%%%%%%%%%%%%%%        %%%%%%%%      %%%%%%%%%%%                             
//          %%%%%%%%%%%%%%%%%%%%%%%%%%%%%      %%%%%%%%%%%%%%%%%       %%%%%%%%       %%%%%%%%%%%                             
//         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%      %%%%%%%%%%%%%%%      %%%%%%%%%       %%%%%%%%%%%%%%%%%%%%%%%%%%%%%            
//        =======================================================================================================             
//       ========================================================================================================             
//      =========================================================================================================             
//     %%%%%%%%%                  %%%%%%%%%             %%%%%%%%%%%%                %%%%%%%%%%%%%%%%%%%%%%%%%%%%              

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <Servo.h>
#include <SoftwareSerial.h>
#include <arduino-timer.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <MicroNMEA.h>


//Pin defs
const int buzzer = 15;
const int led = 16;
const int servo_wire = 17;
const int SDA_PIN = 20;
const int SCL_PIN = 21;
const int VOLT_PIN = 28;

SoftwareSerial XBee(12, 13);
SoftwareSerial OpenLog(4, 5);
Servo release_servo;
Adafruit_BMP3XX bmp;
SFE_UBLOX_GNSS zoe;

char nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));

auto timer = timer_create_default();

String sw_state_list[5] = {"READY", "ASCENDING", "RELEASING", "DESCENDING", "GROUNDED"};
String sw_state = sw_state_list[0];
String pl_state = "N";
int packet_count = 0;
float ground_pressure;
String packet = "";


/*                     |FUNCTIONS|                   */

bool send_packet(void *){
  //TEAM_ID, MISSION_TIME, PACKET_COUNT, SW_STATE, PL_STATE, ALTITUDE, TEMP, VOLTAGE, GPS_LATITUDE, GPS_LONGITUDE
  bmp.performReading();

  packet = "1005," //TEAM_ID
  + cur_time() + "," //MISSION_TIME
  + packet_count + "," //PACKET_COUNT
  + sw_state + "," //SOFTWARE_STATE
  + pl_state + "," //PAYLOAD_STATE
  + bmp.readAltitude(ground_pressure) + "," //ALTITUDE
  + bmp.temperature + "," //TEMPERATURE
  + (float(analogRead(VOLT_PIN)) * (3.3/4095.0) / (1.0/3.0)) + "," //VOLTAGE (analogRead * (max_voltage/12bits) / (ratio of resistors))
  + nmea.getLatitude() + "," //GPS_LATITUDE
  + nmea.getLongitude();  //GPS_LOGITUDE

  XBee.println(packet);
  OpenLog.println(packet);

  packet = "";
  packet_count++;
  nmea.clear();
  return true;
}


String cur_time(){
  unsigned long runMillis = millis();
  unsigned long allSeconds = millis()/1000;
  int h = allSeconds / 3600;
  int secsRemaining = allSeconds % 3600;
  int m = secsRemaining / 60;
  int s = secsRemaining % 60;
  int ms = runMillis % 100;
  char time[11];
  sprintf(time, "%02d:%02d:%02d:%02d", h, m, s, ms);
  return String(time);
}

void SFE_UBLOX_GNSS::processNMEA(char incoming)
{
  //Take the incoming char from the u-blox I2C port and pass it on to the MicroNMEA lib for sentence cracking
  nmea.process(incoming);
}



/*                       |SETUP|                     */
void setup(){
  analogReadResolution(12);

  //OpenLog setup
  OpenLog.begin(9600);


  //XBee setup
  XBee.begin(9600);


  //I2C for BMP & ZOE
  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);
  Wire.begin();


  //ZOE setup
  if (zoe.begin() == false){
    XBee.println("CRITICAL: Could not find a valid GPS device, check wiring!");
    while(1);
  }
  zoe.setProcessNMEAMask(SFE_UBLOX_FILTER_NMEA_GGA);


  //BMP setup
  if (!bmp.begin_I2C()) {
    XBee.println("CRITICAL: Could not find a valid BMP3 sensor, check wiring!");
    while(1);
  }
  for (int i = 0; i < 20; i++){
    bmp.performReading();
    ground_pressure = bmp.pressure / 100;
  }
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  

  //Buzzer and LED setup
  pinMode(led, OUTPUT);
  pinMode(buzzer, OUTPUT);
  

  //Servo setup
  release_servo.attach(servo_wire, 400, 2600);
  release_servo.write(180);


  //Timer setup
  timer.every(1000, send_packet);
}



/*                       |LOOP|                     */
void loop(){
  timer.tick(); // tick the timer

  bmp.performReading();

  if(sw_state == sw_state_list[0]){   //READY
    if(bmp.readAltitude(ground_pressure) >= 20){
      sw_state = sw_state_list[1];
    }
  }

  if(sw_state == sw_state_list[1]){   //ASCENDING
    if(bmp.readAltitude(ground_pressure) >= 490){
      sw_state = sw_state_list[2];
    }
  }

  if(sw_state == sw_state_list[2]){   //DROPPING
    release_servo.write(0);
    pl_state = "R";
    sw_state = sw_state_list[3];
  }

  if(sw_state == sw_state_list[3]){   //DESCENDING
    if(bmp.readAltitude(ground_pressure) <= 20){
      sw_state = sw_state_list[4];
    }
  }

  if(sw_state == sw_state_list[4]){   //GROUNDED
    digitalWrite(led, HIGH);
    digitalWrite(buzzer, HIGH);
  }
}