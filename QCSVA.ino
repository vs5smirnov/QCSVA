#include <WiFiClient.h>
#include <WiFiUdp.h>
#include <WiFiServer.h>
#include <WiFi.h>
#include <SPI.h>
#include <Servo.h>
#include <Wire.h>

#define BUFSIZE 25 
#define PORT 4791         // UDP port
#define MAX_SIGNAL 2000   // Max and min PWM signals (μs)
#define MIN_SIGNAL 1000 
#define MOTOR1_PIN 3      // OC2B
#define MOTOR2_PIN 9      // OC1A
#define MOTOR3_PIN 6      // OC0A
#define MOTOR4_PIN 5      // OC0B
#define EMERDSTPWR 25     // Emergency descent power - 10%
#define THR_ANDROID_MIN 0
#define THR_ANDROID_MAX 255
#define MAX_TILT_THRUST_CORRECTION 1.3 // 30%
#define MAX_TILT_ANGLE
#define PI 3.14159

Servo motor1;             /* Motors (servos), numbering:   1   2   */
Servo motor2;             /*                                \ /    */
Servo motor3;             /*                                / \    */
Servo motor4;             /*                               3   4   */


char ssid[] = "URWA";     // WiFi
char pass[] = "XXXXXXXXXX"; 
char rbuf[BUFSIZE];
char wbuf[BUFSIZE];
boolean   commstart=0;
boolean   badpacket=0;
boolean send_failed=0;
int          status;
WiFiUDP         Udp;

byte     cThrottle;     // Commanded values (from Android)
short         cYaw;
short        cRoll;
short       cPitch;
boolean      cCtrl;     // pitch/roll control on/off
boolean    cCutoff;     // engines cut-off

long  BatLvl =0;
long  WifiAtt=0;

int   gyroResult[3], accelResult[3];  // Gyro/Accel; static accelerometer's biases measured on fixed drone with engines off 
float biasGyroX=0, biasGyroY=0, biasGyroZ=0, biasAccelX=0, biasAccelY=0, biasAccelZ=0;      // accel x=20, z=-13
float pitchGyro;
float pitchAccel;
float rollGyro;
float rollAccel;
float pitchGyroDelta;
float rollGyroDelta;
float pitchGyroRate;
float rollGyroRate;
float pitchError=1.18;//0.34;   // Static accelerometer's errors
float rollError=1.5;//3.99;   // due to not ideal IMU 6DOF chip position on the drone
float Rad=0.01745;

float     Kp=0;      // PID
float     Ki=0;      // 2; 0.8; 0 - test OK
float     Kd=0;
float   sumP=0;
double  sumR=0;
double prevR=0;
double prevP=0;
double  pidP=0;
double  pidR=0;

#define Q1 0.1       // Kalman filter 0.1/0.05/0.005
#define Q2 0.05      
#define Q3 0.005     
#define R1 6000      // 5000
#define R2 715       // 715
struct kalman_data
{
  float x1, x2, x3;
  float p11, p12, p13, p21, p22, p23, p31, p32, p33;
  float q1, q2, q3;
  float r1, r2;
};
kalman_data pitch_data;
kalman_data roll_data;
float pitchPrediction =0; 
float rollPrediction  =0;  

int      ThrBase=0;            // Thrust
int      Thrust1=MIN_SIGNAL;
int      Thrust2=MIN_SIGNAL;
int      Thrust3=MIN_SIGNAL;
int      Thrust4=MIN_SIGNAL;
float    Yaw_factor=1;

boolean            emerdst=0;      // Emergency descent flag
unsigned long     sendtime=0;      // Time since last UDP packet sent (ms)
unsigned short      sendfq=200;    // Frequency of updates to Android (ms)
unsigned long   packettime=0;      // Time since last UDP packet received (ms)
unsigned long   nocommtime=1500;   // Maximum incoming packets absence time (ms)
unsigned long prevlooptime=0;
unsigned long     timeStep;        // Times since last loop ms, s
float            timeStepS;

/**********************************/
/*            S E T U P           */
/**********************************/
void setup() 
{ 
  int totalGyroXValues  =0;
  int totalGyroYValues  =0;
  int totalGyroZValues  =0;
  int i;

  kalman_init(&pitch_data);
  kalman_init(&roll_data);

  pinMode(MOTOR1_PIN, OUTPUT);    
  pinMode(MOTOR2_PIN, OUTPUT);    
  pinMode(MOTOR3_PIN, OUTPUT);    
  pinMode(MOTOR4_PIN, OUTPUT);     

  motor1.attach(MOTOR1_PIN);
  motor2.attach(MOTOR2_PIN);  
  motor3.attach(MOTOR3_PIN);
  motor4.attach(MOTOR4_PIN);

  // disable SD card slot on WiFi shield
  pinMode(4,OUTPUT);      
  digitalWrite(4,HIGH); 

  motorctl(1,MIN_SIGNAL);
  motorctl(2,MIN_SIGNAL);
  motorctl(3,MIN_SIGNAL);
  motorctl(4,MIN_SIGNAL);

  cThrottle=0;
  cYaw     =0;
  cRoll    =0;
  cPitch   =0;
  cCtrl    =0;  
  wbuf[0]='Q';
  wbuf[1]='C';
  wbuf[2]='S';
  wbuf[3]='V';
  wbuf[4]='A';

//  Serial.begin(115200);  /******************************************/

  Wire.begin();            // Gyro/accel settings:
  writeTo(0x53,0x31,0x09); // Set accelerometer to 11bit, +/-4g
  writeTo(0x53,0x2D,0x08); // Set accelerometer to measure mode
  writeTo(0x68,0x16,0x1A); // Set gyro to +/-2000deg/sec (the only option) and 98Hz low pass filter
  writeTo(0x68,0x15,0x01); // Set gyro to 500Hz sample rate -0x01  // 0x09 - 100 MHz


  // Determine zero bias for all axes of both sensors by averaging 50 measurements
  delay(100); // let the drone stabilize after power switch click and sensors initialize
  prevlooptime=millis();     

  for (i=0;i<50;i++) 
  {
    getGyroscopeReadings(gyroResult);     
    getAccelerometerReadings(accelResult);
    totalGyroXValues += gyroResult[0];
    totalGyroYValues += gyroResult[1];
    totalGyroZValues += gyroResult[2];
    delay(25);
  }
  biasGyroX = totalGyroXValues / 50;
  biasGyroY = totalGyroYValues / 50;
  biasGyroZ = totalGyroZValues / 50;

  while (WiFi.status() == WL_NO_SHIELD)
  {    
           // don't continue
  }

  while (WiFi.status() != WL_CONNECTED) 
  {
    status = WiFi.begin(ssid, pass);
    delay(2000);
  }

  while (!Udp.begin(PORT))  { }

}

/**********************************/
/*       M A I N   L O O P        */
/**********************************/
void loop() 
{
  getGyroscopeReadings(gyroResult);      // Read gyro and accelerometer sensors
  timeStep=millis()-prevlooptime;        // measure time
  timeStepS=timeStep/1000.0;
  prevlooptime=millis();    
  getAccelerometerReadings(accelResult);
  pitchAccel =atan2((accelResult[1] - biasAccelY) / 256.0, (accelResult[2] - biasAccelZ) / 256.0) * 360.0 / (2*PI) - pitchError; // убери biasAccel?????????????
  rollAccel  =atan2((accelResult[0] - biasAccelX) / 256.0, (accelResult[2] - biasAccelZ) / 256.0) * 360.0 / (2*PI) - rollError;

   pitchGyroRate=(gyroResult[0] - biasGyroX) / 14.375;
    rollGyroRate=(gyroResult[1] - biasGyroY) / 14.375;
  pitchGyroDelta=pitchGyroRate*timeStepS;
   rollGyroDelta=rollGyroRate *timeStepS;

  // Kalman filter
  kalman_innovate(&pitch_data,pitchAccel, pitchGyroRate,timeStepS); 
  kalman_innovate(&roll_data,  rollAccel,-rollGyroRate, timeStepS);
  pitchPrediction=pitch_data.x1;
  rollPrediction = roll_data.x1;

  rbuf[0]=0;
  if (Udp.parsePacket())
  {
    badpacket=0;
    commstart=1;
    int len=Udp.read(rbuf, BUFSIZE);
    if (len>0) 
    {
      rbuf[len] = 0;
      if (rbuf[0]=='Q' && rbuf[1]=='C' && rbuf[2]=='S' && rbuf[3]=='V' && rbuf[4]=='B')
      {
        packettime=millis();
        emerdst=0;
        cThrottle=rbuf[5];      
        cYaw=rbuf[8] & 0x00FF;
        cYaw-=127;
        Kp=*(float *)&rbuf[10];
        Kd=*(float *)&rbuf[14];        
        Ki=*(float *)&rbuf[18];
        if ( ((short)(rbuf[6] & 0x00FF)>=0) && ((short)(rbuf[6] & 0x00FF)<=180) )
          cRoll=(rbuf[6]& 0x00FF)-90;
        else
          badpacket=1;
        if ( ((short)(rbuf[7] & 0x00FF)>=0) && ((short)(rbuf[7] & 0x00FF)<=180) )
          cPitch=(rbuf[7] & 0x00FF)-90;
        else
          badpacket=1;
        if ((rbuf[9]==0) || (rbuf[9]==1))
          cCtrl=rbuf[9];
        else
          badpacket=1;
        if ((rbuf[22]==0) || (rbuf[22]==1))
          cCutoff=rbuf[22];  
        else
          badpacket=1;
      }
      else 
        badpacket=1;
    } 
  }

  if (!cCtrl)
  {
    cRoll =0;
    cPitch=0;
  }
  else
  {
    cPitch=(cPitch>MAX_TILT_ANGLE)?MAX_TILT_ANGLE:cPitch;
    cRoll = (cRoll>MAX_TILT_ANGLE)?MAX_TILT_ANGLE:cRoll;    
  }
  
  // Emergency descent mode in case of comm loss
  if ((millis()-packettime>=nocommtime)  || (WiFi.status() != WL_CONNECTED))  // 4 - no WiFi or lost WiFi, 3 - OK, (re)connected after both cases
  {
    emerdst=1;  
    cThrottle=EMERDSTPWR;    
  }

  // PID values 
  sumP+=(pitchPrediction+cPitch)*timeStepS; 
  sumR+=(rollPrediction -cRoll) *timeStepS;
  pidP =Kp*(pitchPrediction+cPitch)+ Ki*sumP + Kd*pitchGyroRate;  // Gyro readings * Kd as non-0 gyro is rate of error's change!
  pidR =Kp*(rollPrediction-cRoll)  + Ki*sumR -  Kd*rollGyroRate;

  if (cCutoff) { 
    Thrust1=MIN_SIGNAL; 
    Thrust2=MIN_SIGNAL; 
    Thrust3=MIN_SIGNAL; 
    Thrust4=MIN_SIGNAL; 
  }
  else
  {
    ThrBase=map(cThrottle,THR_ANDROID_MIN,THR_ANDROID_MAX,MIN_SIGNAL,MAX_SIGNAL);    
    float TiltThrCorrection=sqrt(1 + tan(abs(pitchPrediction)*Rad)*tan(abs(pitchPrediction)*Rad) + tan(abs(rollPrediction)*Rad)*tan(abs(rollPrediction)*Rad));
    TiltThrCorrection=TiltThrCorrection > MAX_TILT_THRUST_CORRECTION ? MAX_TILT_THRUST_CORRECTION : TiltThrCorrection;
    ThrBase=ThrBase * TiltThrCorrection;
    
    Thrust1 = ThrBase - (int)cYaw - pidP + pidR;
    Thrust2 = ThrBase + (int)cYaw - pidP - pidR;
    Thrust3 = ThrBase - (int)cYaw + pidP - pidR;
    Thrust4 = ThrBase + (int)cYaw + pidP + pidR;    
  }
  motorctl(1,Thrust1);
  motorctl(2,Thrust2);
  motorctl(3,Thrust3);
  motorctl(4,Thrust4);
    
  if (millis()-sendtime>=sendfq && commstart)
  {
    //BatLvl=readVcc();                 // 4892mV=full; disabled - takes additional 5ms 
    wbuf[0]='Q';      
    wbuf[5]=0;
    wbuf[6]=(short)rollPrediction+90;      
    wbuf[7]=0;
    wbuf[8]=(short)pitchPrediction+90;      
    wbuf[9]=0;  //HDG>>8;               // Heading and altitude not there yet
    wbuf[10]=0; //HDG;    
    wbuf[11]=0; //ALT>>8;
    wbuf[12]=0; //ALT;    
    wbuf[13]=cThrottle;
    wbuf[14]=0;
    wbuf[15]=BatLvl>>8;
    wbuf[16]=BatLvl;  
    wbuf[17]=cYaw;        
    wbuf[18]=cCtrl;  
    wbuf[19]=*(byte *)&timeStep;
    if (Udp.beginPacket(Udp.remoteIP(), Udp.remotePort()))
    {
      Udp.write(wbuf, BUFSIZE);
      if (!Udp.endPacket()) 
      {
        send_failed=1;  
      }
      else
      {
        send_failed=0;
        sendtime=millis();
      }
    }
    else 
    {
      send_failed=1;
    }      
  }
}

void printWifiData() 
{
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
}

long getRSSI() 
{
  uint8_t numNets = WiFi.scanNetworks();  
  long rssi = WiFi.RSSI();
  return rssi;
}

long readVcc() 
{
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif  

  delay(2);                        // Wait for Vref to settle
  ADCSRA |= _BV(ADSC);             // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both

  long result = (high<<8) | low;
  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result;              // Vcc in millivolts
}

void motorctl(unsigned short motor, unsigned short thr)
{
  thr = thr>MAX_SIGNAL?MAX_SIGNAL:thr;
  thr = thr<MIN_SIGNAL?MIN_SIGNAL:thr;  
  if (motor == 1)
    motor1.writeMicroseconds(thr);
  else if (motor == 2)
    motor2.writeMicroseconds(thr);
  else if (motor == 3)
    motor3.writeMicroseconds(thr);
  else if (motor == 4)   
    motor4.writeMicroseconds(thr);
}

void EmergencyDescent ( )  // Not used
{
  // Aviate, Navigate, Communicate! Squawk 7700, comm 121.5
  if (!emerdst)
    emerdst=1;  
}

void writeTo(byte device, byte toAddress, byte val)   //Function for writing a byte to an address on an I2C device
{
  Wire.beginTransmission(device);  
  Wire.write(toAddress);        
  Wire.write(val);        
  Wire.endTransmission();
}

void readFrom(byte device, byte fromAddress, int num, byte result[])   //Function for reading num bytes from addresses on an I2C device
{
  Wire.beginTransmission(device);
  Wire.write(fromAddress);
  Wire.endTransmission();
  Wire.requestFrom((int)device, num);
  int i = 0;
  while(Wire.available()) {
    result[i] = Wire.read();
    i++;
  }
}

void getGyroscopeReadings(int gyroResult[])
{
  byte buffer[6];
  readFrom(0x68,0x1D,6,buffer);
  gyroResult[0] = (((int)buffer[0]) << 8 ) | buffer[1];
  gyroResult[1] = (((int)buffer[2]) << 8 ) | buffer[3];
  gyroResult[2] = (((int)buffer[4]) << 8 ) | buffer[5];
} 

void getAccelerometerReadings(int accelResult[])
{
  byte buffer[6];
  readFrom(0x53,0x32,6,buffer);
  accelResult[0] = (((int)buffer[1]) << 8 ) | buffer[0];
  accelResult[1] = (((int)buffer[3]) << 8 ) | buffer[2];
  accelResult[2] = (((int)buffer[5]) << 8 ) | buffer[4];
}

void kalman_init(struct kalman_data *data)   // Setup the kalman data struct
{
  data->x1 = 0.0f;
  data->x2 = 0.0f;
  data->x3 = 0.0f;

  // Init P to diagonal matrix with large values since
  // the initial state is not known
  data->p11 = 100.0f;
  data->p12 = 0;
  data->p13 = 0;
  data->p21 = 0;
  data->p22 = 100.0f;
  data->p23 = 0;
  data->p31 = 0;
  data->p32 = 0;
  data->p33 = 100.0f;

  data->q1 = Q1;
  data->q2 = Q2;
  data->q3 = Q3;
  data->r1 = R1;
  data->r2 = R2;
}	

void kalman_innovate(kalman_data *data, float z1, float z2, float DT)
{
  float y1, y2;
  float a, b, c;
  float sDet;
  float s11, s12, s21, s22;
  float k11, k12, k21, k22, k31, k32;
  float p11, p12, p13, p21, p22, p23, p31, p32, p33;

  // Step 1
  // x(k) = Fx(k-1) + Bu + w:
  data->x1 = data->x1 + DT*data->x2 - DT*data->x3;
  //x2 = x2;
  //x3 = x3;

  // Step 2
  // P = FPF'+Q
  a = data->p11 + data->p21*DT - data->p31*DT;
  b = data->p12 + data->p22*DT - data->p32*DT;
  c = data->p13 + data->p23*DT - data->p33*DT;
  data->p11 = a + b*DT - c*DT + data->q1;
  data->p12 = b;
  data->p13 = c;
  data->p21 = data->p21 + data->p22*DT - data->p23*DT;
  data->p22 = data->p22 + data->q2;
  //p23 = p23;
  data->p31 = data->p31 + data->p32*DT - data->p33*DT;
  //p32 = p32;
  data->p33 = data->p33 + data->q3; 

  // Step 3
  // y = z(k) - Hx(k)
  y1 = z1-data->x1;
  y2 = z2-data->x2;

  // Step 4
  // S = HPT' + R
  s11 = data->p11 + data->r1;
  s12 = data->p12;
  s21 = data->p21;
  s22 = data->p22 + data->r2;

  // Step 5
  // K = PH*inv(S)
  sDet = 1/(s11*s22 - s12*s21);
  k11 = (data->p11*s22 - data->p12*s21)*sDet;
  k12 = (data->p12*s11 - data->p11*s12)*sDet;
  k21 = (data->p21*s22 - data->p22*s21)*sDet;
  k22 = (data->p22*s11 - data->p21*s12)*sDet;
  k31 = (data->p31*s22 - data->p32*s21)*sDet;
  k32 = (data->p32*s11 - data->p31*s12)*sDet;

  // Step 6
  // x = x + Ky
  data->x1 = data->x1 + k11*y1 + k12*y2;
  data->x2 = data->x2 + k21*y1 + k22*y2;
  data->x3 = data->x3 + k31*y1 + k32*y2;

  // Step 7
  // P = (I-KH)P
  p11 = data->p11*(1.0f - k11) - data->p21*k12;
  p12 = data->p12*(1.0f - k11) - data->p22*k12;
  p13 = data->p13*(1.0f - k11) - data->p23*k12;
  p21 = data->p21*(1.0f - k22) - data->p11*k21;
  p22 = data->p22*(1.0f - k22) - data->p12*k21;
  p23 = data->p23*(1.0f - k22) - data->p13*k21;
  p31 = data->p31 - data->p21*k32 - data->p11*k31;
  p32 = data->p32 - data->p22*k32 - data->p12*k31;
  p33 = data->p33 - data->p23*k32 - data->p13*k31;
  data->p11 = p11; 
  data->p12 = p12; 
  data->p13 = p13;
  data->p21 = p21; 
  data->p22 = p22; 
  data->p23 = p23;
  data->p31 = p31; 
  data->p32 = p32; 
  data->p33 = p33;
}
