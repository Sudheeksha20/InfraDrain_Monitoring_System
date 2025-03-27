#define BLYNK_TEMPLATE_ID "TMPL3Eb7H_vaj"
#define BLYNK_TEMPLATE_NAME "DRAINAGE MONITOR SYSTEM"
#define BLYNK_AUTH_TOKEN "ZfwsK7fhNB4Ji40qX2HpijH3hMInRXCO"

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <OneWire.h>
#include <Wire.h>
#include "DHT.h"
#include "max6675.h"//mx6675 module library 


Adafruit_MPU6050 mpu;

#define DHTTYPE DHT11//dht 11
#define DHTPIN 33
DHT dht(DHTPIN, DHTTYPE);
BlynkTimer timer;



int watersensorPin = 12; //flow sensor
volatile long pulse;
unsigned long lastTime;
float volume;



const float  OffSet = 0.483 ;//ph
float V, P;

float calibration_value = 20.24 - 0.7; //21.34 - 0.7
int phval = 0; 
unsigned long int avgval; 
int buffer_arr[10],temp;
float ph_act;









const int sensorPin = 32;//.................. mq135
const int loadResistance = 1000;



const int sensorPin2 = 34;//.................. mq7
const int loadResistance2 = 1000;


char auth[] = BLYNK_AUTH_TOKEN;

char ssid[] = "TCPROJECT";
char pass[] = "TCPROJECT";






//dht fucntion


void sendSensor()
{
  float h = dht.readHumidity();
  float t = dht.readTemperature(); // or dht.readTemperature(true) for Fahrenheit

  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
  // You can send any value at any time.
  // Please don't send more that 10 values per second.
   Serial.println("DHT TEMP = ");
   Serial.println(t);
   Blynk.virtualWrite(V4, t);
  //Blynk.virtualWrite(V5, h);
}






//ph function

void display_pHValue() {

 for(int i=0;i<10;i++) 
 { 
 buffer_arr[i]=analogRead(35);
 delay(30);
 }
 for(int i=0;i<9;i++)
 {
 for(int j=i+1;j<10;j++)
 {
 if(buffer_arr[i]>buffer_arr[j])
 {
 temp=buffer_arr[i];
 buffer_arr[i]=buffer_arr[j];
 buffer_arr[j]=temp;
 }
 }
 }
 avgval=0;
 for(int i=2;i<8;i++)
 avgval+=buffer_arr[i];
 float volt=(float)avgval*3.3/4096.0/6;  
 //Serial.print("Voltage: ");
 //Serial.println(volt);
  ph_act = -5.70 * volt + calibration_value;
 
 Serial.print("pH Val: ");
 Serial.println(ph_act);
  Blynk.virtualWrite(V1,ph_act);
 delay(1000);
}





//gyro function

void gyro()
{
  if(mpu.getMotionInterruptStatus()) {
    /* Get new sensor events with the readings */
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    /* Print out the values */
 
    Serial.print("GyroX:");
    Serial.print(g.gyro.x);

    Serial.println("");
  // You can send any value at any time.
  // Please don't send more that 10 values per second.


    Blynk.virtualWrite(V3,g.gyro.x);
   
   delay(200);
  }
}





//flose sen
void waterflow() {
  volume = 2.663 * pulse / 1000 * 30;
  if (millis() - lastTime > 2000) {
    pulse = 0;
    lastTime = millis();
  }
  Serial.print(volume);
  Serial.println(" L/m");
   Blynk.virtualWrite(V5, volume);
}

void increase() {
  pulse++;
}



void mq135() 

{
  int sensorValue = analogRead(sensorPin);
  float voltage = sensorValue * (3.3 / 1023); // Assuming 3.3V ADC reference
  float resistance = loadResistance * (5.0 - voltage) / voltage;
  float gasConcentration = pow(10, ((resistance - 11000.0) / 680.0));
    Serial.println("mq 135");
  Serial.println(gasConcentration);
  Blynk.virtualWrite(V12,sensorValue);
  delay(1000);
}




void mq7() 

{
  int sensorValue2 = analogRead(sensorPin2);
  float voltage = sensorValue2 * (3.3 / 1023); // Assuming 3.3V ADC reference
  float resistance2 = loadResistance2 * (5.0 - voltage) / voltage;
  float gasConcentration2 = pow(10, ((resistance2 - 11000.0) / 680.0));
   Serial.println("mq 7");
  Serial.println(gasConcentration2);
   Blynk.virtualWrite(V13, sensorValue2);
  delay(1000);
}


void setup()
{
  // Debug console
  Serial.begin(115200);
  dht.begin();
  Blynk.begin(auth, ssid, pass);
timer.setInterval(5000L, sendSensor);
timer.setInterval(5000L, display_pHValue);
timer.setInterval(6000L, gyro);
timer.setInterval(5000L, waterflow);
timer.setInterval(5000L, mq135);
timer.setInterval(5000L, mq7);

Wire.begin();
Wire.write(0x6B);
Wire.write(0);
Wire.endTransmission(true);
pinMode(watersensorPin, INPUT);
attachInterrupt(digitalPinToInterrupt(watersensorPin), increase, RISING);



 if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(20);
  mpu.setInterruptPinLatch(true); 
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);
  Serial.println("");
  delay(100);

}
  

void loop()
{
  Blynk.run();
  timer.run();
}