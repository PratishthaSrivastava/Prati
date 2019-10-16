#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>
#include <Wire.h>
#include <SPI.h>
#include "SH1106.h"
#include "SH1106Ui.h"
#include "SH1106Ui.h"
#include "images.h"
#define _baudrate   9600
/*********Hardware Related Macros*************/
#define         MQ_PIN                       (A0)     //define which analog input channel you are going to use
#define         MQ9PIN                       (A0)
#define         D0                           (16) //
#define         D1                           (5)
#define         D2                           (4)
#define         RL_VALUE                     (5)     //define the load resistance on the board, in kilo ohms
#define         RO_CLEAN_AIR_FACTOR          (9.83)  //RO_CLEAR_AIR_FACTOR=(Sensor resistance in clean air)/RO,
#define         RO_CLEAN_AIR_FACTOR_MQ9      (9.799)                                                     //which is derived from the chart in datasheet
#define         RL_VALUE_MQ9                 (1)
#define         measurePin                  (A0)
/********Software Related Macros*************/
#define         CALIBARAION_SAMPLE_TIMES     (50)    //define how many samples you are going to take in the calibration phase
#define         CALIBRATION_SAMPLE_INTERVAL  (500)   //define the time interal(in milisecond) between each samples in the
//cablibration phase
#define         READ_SAMPLE_INTERVAL         (50)    //define how many samples you are going to take in normal operation
#define         READ_SAMPLE_TIMES            (30)     //define the time interal(in milisecond) between each samples in 

/*******Application Related Macros***********/
#define         GAS_LPG                      (0)
#define         GAS_CO                       (1)
#define         GAS_SMOKE                    (2)
#define         GAS_CARBON_MONOXIDE          (3)
#define         GAS_METHANE                  (4)
#define         myPeriodic                    15
/**********Globals****************/
#define OLED_RESET  D3   // RESET
#define OLED_DC     D4   // Data/Command
#define OLED_CS     15   // Chip select
int upload_count=0;
float           LPGCurve[3]  =  {2.3, 0.21, -0.47}; //two points are taken from the curve.
//with these two points, a line is formed which is "approximately equivalent"
//to the original curve.
//data format:{ x, y, slope}; point1: (lg200, 0.21), point2: (lg10000, -0.59)
float           COCurve[3]  =  {2.3, 0.72, -0.34};  //two points are taken from the curve.
//with these two points, a line is formed which is "approximately equivalent"
//to the original curve.
//data format:{ x, y, slope}; point1: (lg200, 0.72), point2: (lg10000,  0.15)
float           SmokeCurve[3] = {2.3, 0.53, -0.44}; //two points are taken from the curve.
//with these two points, a line is formed which is "approximately equivalent"
//to the original curve.
//data format:{ x, y, slope}; point1: (lg200, 0.53), point2: (lg10000,  -0.22)
float           Ro           =  10;                 //Ro is initialized to 10 kilo ohms
float           Ro_9         =  20;

float GAS_S = 0;      float CO = 0;
int count = 0;
int sent = 0;
unsigned int pm1 = 0;
double pm2_5 = 0;
unsigned int pm10 = 0;

//*-- IoT Information
const char* MY_SSID = "Your Wifi name";
const char* MY_PWD = "Your Wifi Password";
const char* server = "api.thingspeak.com";
// 使用 GET 傳送資料的格式
// GET /update?key=[THINGSPEAK_KEY]&field1=[data 1]&filed2=[data 2]...;
String apiKey = "Your Write API KEY";
SH1106 display(true, OLED_RESET, OLED_DC, OLED_CS);
SH1106Ui ui     ( &display );

bool drawFrame2(SH1106 *display, SH1106UiState* state, int x, int y) {
  display->setFont(ArialMT_Plain_24);
  display->setTextAlignment(TEXT_ALIGN_LEFT);
  display->drawString(32 + x, 0 + y, "PM 2.5");
  display->drawString(10 + x, 32 + y, String(pm2_5) + "  ppm");
}
bool drawFrame3(SH1106 *display, SH1106UiState* state, int x, int y) {
  display->setFont(ArialMT_Plain_24);
  display->setTextAlignment(TEXT_ALIGN_LEFT);
  display->drawString(30 + x, 0 + y, "Smoke");
  display->drawString(10 + x, 32 + y, String(GAS_S) + "  ppm");
}

bool drawFrame4(SH1106 *display, SH1106UiState* state, int x, int y) {
  display->setFont(ArialMT_Plain_24);
  display->setTextAlignment(TEXT_ALIGN_LEFT);
  display->drawString(38 + x, 0 + y, "CO");
  display->drawString(10 + x, 32 + y, String(CO) + "  ppm");
}

// how many frames are there?
int frameCount = 3;
// this array keeps function pointers to all frames
// frames are the single views that slide from right to left

bool (*frames[])(SH1106 *display, SH1106UiState* state, int x, int y) = {drawFrame2, drawFrame3, drawFrame4 };

int  MQGetPercentage(float rs_ro_ratio, float *pcurve)
{
  return (pow(10, ( ((log(rs_ro_ratio) - pcurve[1]) / pcurve[2]) + pcurve[0])));
}

int MQ9GetGasPercentage(float rs_ro_ratio, int gas_id)
{
  if ( gas_id == GAS_CARBON_MONOXIDE ) {
    return (pow(10, ((-2.199 * (log10(rs_ro_ratio))) + 2.766 )));
  } else if ( gas_id == GAS_METHANE ) {
    return (pow(10, (-0.670 * pow((log10(rs_ro_ratio)), 2) - 2.399 * (log10(rs_ro_ratio)) + 3.650)));
  }
  return 0;
}

float MQ2ResistanceCalculation(int raw_adc)
{
  return ( ((float)RL_VALUE * (1023 - raw_adc) / raw_adc));
}
float MQ9ResistanceCalculation(int raw_adc)
{
  return ( ((float)RL_VALUE_MQ9 * (1023 - raw_adc) / raw_adc));
}

float MQ9Calibration(int mq_pin)
{
  int i;
  float RS_AIR_val = 0, r0;

  for (i = 0; i < CALIBARAION_SAMPLE_TIMES; i++) {               //take multiple samples
    RS_AIR_val += MQ9ResistanceCalculation(analogRead(mq_pin));
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  RS_AIR_val = RS_AIR_val / CALIBARAION_SAMPLE_TIMES;            //calculate the average value

  r0 = RS_AIR_val / RO_CLEAN_AIR_FACTOR_MQ9;                    //RS_AIR_val divided by RO_CLEAN_AIR_FACTOR yields the Ro
  //according to the chart in the datasheet
  Serial.print("MQ9 done.\n");
  return r0;
}

float MQ2Calibration(int mq_pin)
{
  int i;
  float val = 0;

  for (i = 0; i < CALIBARAION_SAMPLE_TIMES; i++) {      //take multiple samples
    val += MQ2ResistanceCalculation(analogRead(mq_pin));
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  val = val / CALIBARAION_SAMPLE_TIMES;                 //calculate the average value

  val = val / RO_CLEAN_AIR_FACTOR;                      //divided by RO_CLEAN_AIR_FACTOR yields the Ro
  //according to the chart in the datasheet
  Serial.print("MQ2 done.\n");
  return val;
}
void connectWifi()
{
  Serial.print("Connecting to " + *MY_SSID);
  WiFi.begin(MY_SSID, MY_PWD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("Connected");
  Serial.println("");
}
void setup() {

  Serial.begin( _baudrate );
  pinMode(D0, OUTPUT);
  pinMode(D1, OUTPUT);
  pinMode(D2, OUTPUT);
  ui.setTargetFPS(30);
  ui.setActiveSymbole(activeSymbole);
  ui.setInactiveSymbole(inactiveSymbole);

  // You can change this to
  // TOP, LEFT, BOTTOM, RIGHT
  ui.setIndicatorPosition(BOTTOM);

  // Defines where the first frame is located in the bar.
  ui.setIndicatorDirection(LEFT_RIGHT);
  // You can change the transition that is used
  // SLIDE_LEFT, SLIDE_RIGHT, SLIDE_TOP, SLIDE_DOWN
  ui.setFrameAnimation(SLIDE_DOWN);
  // Add frames
  ui.setFrames(frames, frameCount);
  // Inital UI takes care of initalising the display too.
  ui.init();
  //Calibrating();

}
void Calibrating() {

  Serial.print("Calibrating...\n");
  digitalWrite(D0, 0);
  digitalWrite(D1, 1);
  digitalWrite(D2, 0);
  Ro = MQ2Calibration(MQ_PIN);

  delay(100);
  digitalWrite(D0, 1);
  digitalWrite(D1, 0);
  digitalWrite(D2, 0);
  Ro_9 = MQ9Calibration(MQ9PIN);

  Serial.print("Ro=");
  Serial.print(Ro);
  Serial.print("kohm\t");
  Serial.print("Ro_9=");
  Serial.print(Ro_9);
  Serial.print("kohm");
  Serial.print("\n");
}
void SentOnCloud( String S, String CO, String PM_1, String PM_25, String PM_10)
{
  WiFiClient client;
  if (client.connect(server, 80))
  { //use ip 184.106.153.149 or api.thingspeak.com
    Serial.println("WiFi Client connected ");

    String postStr = apiKey;
    postStr += "&field1=";
    postStr += S;
    postStr += "&field2=";
    postStr += CO;
    postStr += "&field3=";
    postStr += PM_1;
    postStr += "&field4=";
    postStr += PM_25;
    postStr += "&field5=";
    postStr += PM_10;
    postStr += "\r\n\r\n";

    client.print("POST /update HTTP/1.1\n");
    client.print("Host: api.thingspeak.com\n");
    client.print("Connection: close\n");
    client.print("X-THINGSPEAKAPIKEY: " + apiKey + "\n");
    client.print("Content-Type: application/x-www-form-urlencoded\n");
    client.print("Content-Length: ");
    client.print(postStr.length());
    client.print("\n\n");
    client.print(postStr);
    delay(1000);

  }//end if
}

float MQ2Read(int mq_pin)
{
  int i;
  float rs = 0;
  float j;
  for (i = 0; i < READ_SAMPLE_TIMES; i++) {
    rs += MQ2ResistanceCalculation(analogRead(mq_pin));
    j = analogRead(mq_pin);
    delay(READ_SAMPLE_INTERVAL);
  }
  rs = rs / READ_SAMPLE_TIMES;

  return rs;
}

float MQ9Read(int mq_pin)
{
  int i;
  float rs = 0;

  for (i = 0; i < READ_SAMPLE_TIMES; i++) {
    rs += MQ9ResistanceCalculation(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
  }

  rs = rs / READ_SAMPLE_TIMES;

  return rs;
}

int MQGetGasPercentage(float rs_ro_ratio, int gas_id)
{
  if ( gas_id == GAS_LPG ) {
    return MQGetPercentage(rs_ro_ratio, LPGCurve);
  } else if ( gas_id == GAS_CO ) {
    return MQGetPercentage(rs_ro_ratio, COCurve);
  } else if ( gas_id == GAS_SMOKE ) {
    return MQGetPercentage(rs_ro_ratio, SmokeCurve);
  }
  return 0;
}

void loop() {
  int index = 0;
  char value;
  char previousValue;
  GAS_S = MQGetGasPercentage(MQ2Read(MQ_PIN) / Ro, GAS_SMOKE);
  CO = MQGetGasPercentage(MQ9Read(MQ9PIN) / Ro, GAS_CARBON_MONOXIDE) ;

  while (Serial.available()) {
    value = Serial.read();
    if ((index == 0 && value != 0x42) || (index == 1 && value != 0x4d)) {
      Serial.println("Cannot find the data header.");
      break;
    }
    if (index == 4 || index == 6 || index == 8 || index == 10 || index == 12 || index == 14) {
      previousValue = value;
    }
    else if (index == 5) {
      pm1 = 256 * previousValue + value;
      Serial.print("{ ");
      Serial.print("\"pm1\": ");
      Serial.print(pm1);
      Serial.print(", ");
    }
    else if (index == 7) {
      pm2_5 = 256 * previousValue + value;
      Serial.print("\"pm2_5\": ");
      Serial.print(pm2_5);
      Serial.print(", ");
    }
    else if (index == 9) {
      pm10 = 256 * previousValue + value;
      Serial.print("\"pm10\": ");
      Serial.print(pm10);
    } else if (index > 15) {
      break;
    }
    index++;
  }
  while (Serial.available()) Serial.read();
  Serial.println(" }");
  digitalWrite(D0, 0);
  digitalWrite(D1, 1);
  digitalWrite(D2, 0);  //on  MQ2
  Serial.println("MQ2 value:");
  Serial.print("Smoke: ");  Serial.print(GAS_S);   Serial.println( "ppm" );
  digitalWrite(D0, 1);  //on MQ9
  digitalWrite(D1, 0);
  digitalWrite(D2, 0);
  Serial.println("MQ9 value:");
  Serial.print("CARBON_MONOXIDE:");  Serial.print(CO);  Serial.println( "ppm" );
  upload_count++;
  if(upload_count==10){
  connectWifi();
  SentOnCloud(String(GAS_S), String(CO), String(pm1), String(pm2_5), String(pm10));
  WiFi.disconnect();
  upload_count=0;
  }
  int remainingTimeBudget = ui.update();
 
}
