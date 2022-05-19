#include <SPI.h>
#include <SD.h>
#include <dht.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <SFE_BMP180.h>
int BH1750address = 0x23;
byte buff[2];
SoftwareSerial pmsSerial1(2, 3);
dht DHT;
#define DHT11_PIN 7
int ledpin = 8; 
SFE_BMP180 pressure;
File myFile;

struct pms5003data {
  uint16_t framelen;
  uint16_t pm10_standard, pm25_standard, pm100_standard;
  uint16_t pm10_env, pm25_env, pm100_env;
  uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;
  uint16_t unused;
  uint16_t checksum;
};
 
struct pms5003data data;

void setup()
{  
  Serial.begin(9600);
  pmsSerial1.begin(9600);
  Wire.begin();
  if (SD.begin(4))
    Serial.println("SD card is present & ready");
  pinMode(ledpin, OUTPUT);         
  if (pressure.begin())
  {
  Serial.print("pressure valid");
  delay(1000);
  }
  myFile = SD.open("test.txt", FILE_WRITE);
  if(myFile)
  {
    myFile.print(", ");
    myFile.print(", ");
    myFile.print(", ");
    myFile.print(", ");
    myFile.print(", ");
    myFile.print(", ");
    myFile.print(", ");
    myFile.print(", ");
    myFile.print(", ");
    myFile.print(", ");
    myFile.println();
    myFile.close();
  }
  
}

float sensor_volt;
float RS_gas; 
float R0,ratio;
int R2 = 2000;
int chk;
float temp,hum;
float pm1o,pm2o,pm10o;


void loop()
{
  digitalWrite(ledpin, HIGH);    // turn the LED ON by making the voltage HIGH
  delay(1000);
 
  //reading the data
  chk = DHT.read11(DHT11_PIN);
  temp = DHT.temperature;
  hum = DHT.humidity;
  
  if (readPMSdata(&pmsSerial1)) 
  {
    pm1o = data.pm10_standard;
    pm2o = data.pm25_standard;
    pm10o = data.pm100_standard; 
  }

//CO
// for calculating R0 value
  int sensorValue = analogRead(A0);
  sensor_volt=(float)sensorValue/1024*5.0;
  RS_gas = ((5.0 * R2)/sensor_volt) - R2;
  R0 = RS_gas / 1;

// for calculating ppm
  sensorValue = analogRead(A0);
  sensor_volt = sensorValue/1024*5.0;
  RS_gas = (5.0-sensor_volt)/sensor_volt;
  ratio = RS_gas/R0; //Replace R0 with the value found using the sketch above
  float x = 1538.46 * ratio;
  float ppm = pow(x,-1.709);
  
  //PRESSURE
  char status; double P,T;
  status = pressure.startPressure(3);
      if (status != 0)
      {
        
        delay(status);
        status = pressure.getPressure(P,T);
        if (status != 0)
        {
          if(temp<0 || hum<0 || pm1o<0 || pm2o<0 || pm10o<0 || R0<0 || temp>50 || hum>100  || pm1o>10000 || pm2o>10000 || pm10o>10000)//|| P>100000   P<0 ||
          {
            while(1)
            {
              digitalWrite(ledpin, LOW );    // turn the LED off by making the voltage LOW
              delay(1000);
              digitalWrite(ledpin, HIGH);    // turn the LED ON by making the voltage HIGH
              delay(1000);
            }
          }
          Serial.println("Temp:"+String(temp));
          Serial.println("Hum(%):"+String(hum));
          Serial.println("R0:"+String(R0));
          Serial.println("CO(ppm):"+String(ppm));
          Serial.println("Analog:"+String(sensorValue));
          Serial.println("Pressure(mb):"+String(P,2));
          Serial.println("Pressure(Hg):"+String(P*0.0295333727,2));
          Serial.print("PM1.0:"+String(pm1o));
          Serial.print("  PM2.5:"+String(pm2o));
          Serial.print("  PM10:"+String(pm10o));
          
        }
      }
  int i;
  uint16_t value=0;
  BH1750_Init(BH1750address);
  delay(200);
 
  if(2==BH1750_Read(BH1750address))
  {
    value=((buff[0]<<8)|buff[1])/1.2;
    Serial.println("Light:"+String(value));     
    //Serial.println("[lux]"); 
  }
  
delay(15000);
  

  myFile = SD.open("test.txt", FILE_WRITE);
  if(myFile)
  {
    Serial.print("Writing to test.txt...");
    myFile.print(pm1o);
    myFile.print(", ");
    myFile.print(pm2o);
    myFile.print(", ");
    myFile.print(pm10o);
    myFile.print(", ");
    myFile.print(R0);
    myFile.print(", ");
    myFile.print(ppm);
    myFile.print(", ");
    myFile.print(sensorValue);
    myFile.print(", ");
    myFile.print(temp);
    myFile.print(", ");
    myFile.print(hum);
    myFile.print(", ");
    myFile.print(value);
    myFile.print(", ");
    myFile.print(P,2);
    myFile.print(", ");
    myFile.print(P*0.0295333727,2);
    myFile.print(", ");
    myFile.println();
    myFile.close();
  }
  else
  {
    while(1)
            {
              digitalWrite(ledpin, LOW );    // turn the LED off by making the voltage LOW
              delay(1000);
              digitalWrite(ledpin, HIGH);    // turn the LED ON by making the voltage HIGH
              delay(1000);
            }
  }
  Serial.println("Done");
}

boolean readPMSdata(Stream *s) 
{
  if (! s->available())
    return false;

  if (s->peek() != 0x42) 
  {
    s->read();
    return false;
  }
 
  if (s->available() < 32) 
      return false;
      
  uint8_t buffer[32];    
  uint16_t sum = 0;
  s->readBytes(buffer, 32);
 
  for (uint8_t i=0; i<30; i++) 
      sum += buffer[i];
 
  uint16_t buffer_u16[15];
  for (uint8_t i=0; i<15; i++) 
  {
    buffer_u16[i] = buffer[2 + i*2 + 1];
    buffer_u16[i] += (buffer[2 + i*2] << 8);
  }
 
  memcpy((void *)&data, (void *)buffer_u16, 30);
 
  if (sum != data.checksum) 
  {
    Serial.println("Checksum failure");
    return false;
  }
  return true;
}

int BH1750_Read(int address) 
{
  int i=0;
  Wire.beginTransmission(address);
  Wire.requestFrom(address, 2);
  while(Wire.available()) 
  {
    buff[i] = Wire.read();
    i++;
  }
  Wire.endTransmission();  
  return i;
}
 
void BH1750_Init(int address) 
{
  Wire.beginTransmission(address);
  Wire.write(0x10);
  Wire.endTransmission();
}
