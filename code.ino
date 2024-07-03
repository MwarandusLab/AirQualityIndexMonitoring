#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define DHTPIN 8
#define DHTTYPE DHT11


#define MQ_PIN (0)                  //define which   analog input channel you are going to use
#define RL_VALUE (5)                //define the load resistance on the board, in kilo ohms
#define RO_CLEAN_AIR_FACTOR (9.83)  //RO_CLEAR_AIR_FACTOR=(Sensor resistance in clean air)/RO, \
                                    //which is derived from the   chart in datasheet

/**********************Software Related Macros***********************************/
#define CALIBARAION_SAMPLE_TIMES (50)      //define how many samples you are   going to take in the calibration phase
#define CALIBRATION_SAMPLE_INTERVAL (500)  //define the time interal(in milisecond) between each samples in the \
                                           //cablibration phase
#define READ_SAMPLE_INTERVAL (50)          //define how many samples you are   going to take in normal operation
#define READ_SAMPLE_TIMES (5)              //define the time interal(in milisecond) between each samples in

/*********************Application Related Macros*********************************/
#define GAS_LPG (0)
#define GAS_CO (1)
#define GAS_SMOKE (2)

/****************************Globals**********************************************/
float LPGCurve[3] = { 2.3, 0.21, -0.47 };    //two points are taken from the curve.
                                             //with these two points,   a line is formed which is "approximately equivalent"
                                             //to   the original curve.
                                             //data   format:{ x, y, slope}; point1: (lg200, 0.21), point2: (lg10000, -0.59)
float COCurve[3] = { 2.3, 0.72, -0.34 };     //two points are taken from the curve.
                                             //with these two points,   a line is formed which is "approximately equivalent"
                                             //to   the original curve.
                                             //data   format:{ x, y, slope}; point1: (lg200, 0.72), point2: (lg10000,  0.15)
float SmokeCurve[3] = { 2.3, 0.53, -0.44 };  //two points are taken from the curve.
                                             //with these two points,   a line is formed which is "approximately equivalent"
                                             //to   the original curve.
                                             //data   format:{ x, y, slope}; point1: (lg200, 0.53), point2: (lg10000,  -0.22)
float Ro = 10;                               //Ro is initialized to 10 kilo ohms


int measurePin = A5;
int ledPower = 12;
int Buzzer = 7;
unsigned int samplingTime = 280;
unsigned int deltaTime = 40;
unsigned int sleepTime = 9680;

float voMeasured = 0;
float calcVoltage = 0;
float dustDensity = 0;

DHT_Unified dht(DHTPIN, DHTTYPE);

uint32_t delayMS;

void setup() {
  Serial.begin(9600);
  pinMode(Buzzer, OUTPUT);
  pinMode(ledPower, OUTPUT);
  digitalWrite(Buzzer, LOW);

  Ro = MQCalibration(MQ_PIN);  //Calibrating the sensor. Please   make sure the sensor is in clean air          //when   you perform the calibration
  Serial.print("Calibration   is done...");
  Serial.print("Ro=");
  Serial.print(Ro);
  Serial.print("kohm");
  Serial.print("\n");
  
  // Initialize device.
  dht.begin();
  Serial.println(F("DHTxx Unified Sensor Example"));
  // Print temperature sensor details.
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  Serial.println(F("------------------------------------"));
  Serial.println(F("Temperature Sensor"));
  Serial.print(F("Sensor Type: "));
  Serial.println(sensor.name);
  Serial.print(F("Driver Ver:  "));
  Serial.println(sensor.version);
  Serial.print(F("Unique ID:   "));
  Serial.println(sensor.sensor_id);
  Serial.print(F("Max Value:   "));
  Serial.print(sensor.max_value);
  Serial.println(F("°C"));
  Serial.print(F("Min Value:   "));
  Serial.print(sensor.min_value);
  Serial.println(F("°C"));
  Serial.print(F("Resolution:  "));
  Serial.print(sensor.resolution);
  Serial.println(F("°C"));
  Serial.println(F("------------------------------------"));
  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);
  Serial.println(F("Humidity Sensor"));
  Serial.print(F("Sensor Type: "));
  Serial.println(sensor.name);
  Serial.print(F("Driver Ver:  "));
  Serial.println(sensor.version);
  Serial.print(F("Unique ID:   "));
  Serial.println(sensor.sensor_id);
  Serial.print(F("Max Value:   "));
  Serial.print(sensor.max_value);
  Serial.println(F("%"));
  Serial.print(F("Min Value:   "));
  Serial.print(sensor.min_value);
  Serial.println(F("%"));
  Serial.print(F("Resolution:  "));
  Serial.print(sensor.resolution);
  Serial.println(F("%"));
  Serial.println(F("------------------------------------"));
  // Set delay between sensor readings based on sensor details.
  delayMS = sensor.min_delay / 1000;
}

void loop() {
  DHT11_Sensor();
  Dust_Sensor();
  MQ_2_Sensor();
  if(MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_LPG) > 10 || MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_CO) > 10 || MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_SMOKE) > 10){
    digitalWrite(Buzzer, HIGH);

  }else if((MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_LPG) < 10 || MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_CO) < 10 || MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_SMOKE) < 10) && (voMeasured < 500)){
    digitalWrite(Buzzer, LOW);
  }
  delay(1000);
}
void DHT11_Sensor() {
  delay(delayMS);
  // Get temperature event and print its value.
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println(F("Error reading temperature!"));
  } else {
    Serial.print(F("Temperature: "));
    Serial.print(event.temperature);
    Serial.println(F("°C"));
  }
  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println(F("Error reading humidity!"));
  } else {
    Serial.print(F("Humidity: "));
    Serial.print(event.relative_humidity);
    Serial.println(F("%"));
  }
}
void Dust_Sensor() {
  digitalWrite(ledPower, LOW);
  delayMicroseconds(samplingTime);

  voMeasured = analogRead(measurePin);

  delayMicroseconds(deltaTime);
  digitalWrite(ledPower, HIGH);
  delayMicroseconds(sleepTime);

  calcVoltage = voMeasured * (5.0 / 1024);
  dustDensity = 0.17 * calcVoltage - 0.1;

  if (dustDensity < 0) {
    dustDensity = 0.00;
  }

  Serial.println("Raw Signal Value (0-1023):");
  Serial.println(voMeasured); // Call this value and the data will be retrived

  if (voMeasured > 500) {
    digitalWrite(Buzzer, HIGH);
    Serial.println("Excess Dust Detected");
  } else {
    digitalWrite(Buzzer, LOW);
    Serial.println("Normal");
  }

  delay(1000);
}
void MQ_2_Sensor(){
   Serial.print("LPG:"); 
   Serial.print(MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_LPG)   );
   Serial.print( "ppm" );
   Serial.print("    ");   
   Serial.print("CO:");   
   Serial.print(MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_CO) );
   Serial.print(   "ppm" );
   Serial.print("    ");   
   Serial.print("SMOKE:"); 
    Serial.print(MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_SMOKE) );
   Serial.print(   "ppm" );
   Serial.print("\n");
}
float MQResistanceCalculation(int raw_adc)
{
  return ( ((float)RL_VALUE*(1023-raw_adc)/raw_adc));
}
float MQCalibration(int mq_pin)
{
  int i;
  float val=0;
 
   for (i=0;i<CALIBARAION_SAMPLE_TIMES;i++) {            //take multiple samples
     val += MQResistanceCalculation(analogRead(mq_pin));
    delay(CALIBRATION_SAMPLE_INTERVAL);
   }
  val = val/CALIBARAION_SAMPLE_TIMES;                   //calculate the average   value
 
  val = val/RO_CLEAN_AIR_FACTOR;                        //divided   by RO_CLEAN_AIR_FACTOR yields the Ro 
                                                        //according   to the chart in the datasheet 
 
  return val; 
}
/***************************   MQRead *******************************************
Input:   mq_pin - analog   channel
Output:  Rs of the sensor
Remarks: This function use MQResistanceCalculation   to caculate the sensor resistenc (Rs).
         The Rs changes as the sensor   is in the different consentration of the target
         gas. The sample times   and the time interval between samples could be configured
         by changing   the definition of the macros.
**********************************************************************************/   
float MQRead(int mq_pin)
{
  int i;
  float rs=0;
 
  for (i=0;i<READ_SAMPLE_TIMES;i++)   {
    rs += MQResistanceCalculation(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
   }
 
  rs = rs/READ_SAMPLE_TIMES;
 
  return rs;  
}
 
/***************************   MQGetGasPercentage ********************************
Input:   rs_ro_ratio -   Rs divided by Ro
         gas_id      - target gas type
Output:  ppm of the   target gas
Remarks: This function passes different curves to the MQGetPercentage   function which 
         calculates the ppm (parts per million) of the target   gas.
**********************************************************************************/   
int MQGetGasPercentage(float rs_ro_ratio, int gas_id)
{
  if ( gas_id   == GAS_LPG ) {
     return MQGetPercentage(rs_ro_ratio,LPGCurve);
  } else   if ( gas_id == GAS_CO ) {
     return MQGetPercentage(rs_ro_ratio,COCurve);
   } else if ( gas_id == GAS_SMOKE ) {
     return MQGetPercentage(rs_ro_ratio,SmokeCurve);
   }    
 
  return 0;
}
 
/***************************  MQGetPercentage   ********************************
Input:   rs_ro_ratio - Rs divided by Ro
          pcurve      - pointer to the curve of the target gas
Output:  ppm of   the target gas
Remarks: By using the slope and a point of the line. The x(logarithmic   value of ppm) 
         of the line could be derived if y(rs_ro_ratio) is provided.   As it is a 
         logarithmic coordinate, power of 10 is used to convert the   result to non-logarithmic 
         value.
**********************************************************************************/   
int  MQGetPercentage(float rs_ro_ratio, float *pcurve)
{
  return (pow(10,(   ((log(rs_ro_ratio)-pcurve[1])/pcurve[2]) + pcurve[0])));
}

