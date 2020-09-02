//control of vacuum to picker


#include <PID_v1.h>
#include <SPI.h>
#include "mcp_can.h"

const int vacuum_limit=50;   //sends ok to picker if vacuum below limit
int pump_on=1;


const int motor_pin=3; //PWM out to pumps
const int enable = 4;
const int pressure_in=A1;
double V_out;

long timer=0;
int interval=500; //interval to send pressure status

double pwm_to_motor;
double set_value;
double pressure_kPa;

unsigned char pressure_ok[1]={1};
unsigned char presure_high[1]={0};

double Kp=2.0; //Initial Proportional Gain 
double Ki=5.0; //Initial Integral Gain 
double Kd=1.0;  //Initial Differential Gain 

//Can-control
const int spiCSPin = 10;
MCP_CAN CAN(spiCSPin);


unsigned char len = 0;
unsigned char CAN_input[2];
int CAN_in_int[2];



PID myPID(&pressure_kPa, &pwm_to_motor, &set_value, Kp, Ki, Kd, REVERSE); // This sets up our PID Loop (Input, Output, Setpoint, Kp, Ki, Kd)

void setup() {
  
Serial.begin(115200);
Serial.println(" - Vacuum 28 aug 2020");
set_value=30;
myPID.SetMode(AUTOMATIC);


   while (CAN_OK != CAN.begin(CAN_500KBPS , MCP_8MHz))
    {
        Serial.println("CAN BUS Init Failed");
        delay(100);
    }
    Serial.println("CAN BUS  Init OK!");

 CAN.init_Mask(0, 0, 0x3ff);                         // there are 2 mask in mcp2515, you need to set both of them
    CAN.init_Mask(1, 0, 0x3ff);
    CAN.init_Filt(0, 0, 0x06);                      //Filter for messages from Petter
 
}


void loop() {

//recieve can message to start pump

   if(CAN_MSGAVAIL == CAN.checkReceive())
          {  
           CAN.readMsgBuf(&len, CAN_input);
           unsigned long canId = CAN.getCanId();
           for(int i = 0; i<len; i++)
                      {
                       
                       CAN_in_int[i]=CAN_input[i]; 
                        CAN_input[i]=0; //set array to NULL
                       //Serial.print("Can input after reset = "); Serial.println(CAN_input[i]);
                      }
                      
                           if (CAN_in_int[0]==7)            //7 start/stop command
                           {
                                if (CAN_in_int[1]==1) //start pump
                                   {       
                                    pump_on=1;
                                   }
                               if (CAN_in_int[1]==0) //stop pump
                                   {       
                                    pump_on=0;
                                   }                         
                            }
          }

digitalWrite(enable, pump_on);


V_out=analogRead(pressure_in);
pressure_kPa=((V_out/1024.0)+0.095)/0.009;

if (millis()-timer>interval)
{
  SEND_Vacuum_State();
  Serial.println(pressure_kPa);
  timer=millis();
}


myPID.Compute();
analogWrite(motor_pin,pwm_to_motor);

}
