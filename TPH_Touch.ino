/*
 * Project TPH_Touch
 * Description:  BME280 TPH sensor with nextion display
 * Author:  PJB
 * Date:  06/30/21
 */

#include "Particle.h"
#include "Nextion.h"
#include "math.h"
#include "Adafruit_BME280.h"
//#include "JsonParserGeneratorRK.h"    //install it JsonParserGeneratorRK
#include "ThingSpeak.h"

SYSTEM_THREAD(ENABLED);

TCPClient client;

USARTSerial& nexSerial = Serial1;
#define DEBUG_SERIAL_ENABLE
#define dbSerial Serial

#define UPDATE_INTERVAL 10000  //1 sec = 1000 millis

Adafruit_BME280 bme;

float BME_TC, BME_PPa, BME_RH, BME_TF, BME_Ppsi, BME_HI, BME_DPF;

//timegrab
int hr_i, min_i, ampm_i, weekday_i, month_i, day_i, min_time, min_last;
String ampm_s, weekday_s, month_s;

int updateInterval;

//thingspeak
unsigned long myTSChannel1 = 1443781;       
const char * myTSWriteAPIKey1 = "N64Y55UZWC5H022G";

SerialLogHandler logHandler(LOG_LEVEL_INFO);

//page0
NexNumber n000 = NexNumber(0, 1, "n000");       //hour
NexNumber n001 = NexNumber(0, 2, "n001");       //minute
NexText t000 = NexText(0, 3, "t000");           //AM-PM 
NexText t001 = NexText(0, 4, "t001");           //weekday 
NexText t002 = NexText(0, 5, "t002");           //month 
NexNumber n002 = NexNumber(0, 6, "n002");       //day-date

//page 1
NexNumber n100 = NexNumber(1, 16, "n100");    //Integer BME280 Temperature x 10 (for x0 to be 0.1 precision)
NexNumber n101 = NexNumber(1, 17, "n101");    //Integer BME280 Pressure x 1000 (for x1 to be 0.001 precision)
NexNumber n102 = NexNumber(1, 18, "n102");    //Integer BME280 Humidity x 10 (for x2 to be 0.1 precision)
NexNumber n103 = NexNumber(1, 19, "n103");    //Integer BME280 Heat Index x 10 (for x3 to be 0.1 precision)
NexNumber n104 = NexNumber(1, 21, "n104");    //Integer BME280 Dew Point x 10 (for x4 to be 0.1 precision)    

NexTouch *nex_listen_list[] = 
{ 
  NULL
};


void setup()
{  
  Serial.begin(9600);
  delay(100);

  Serial1.begin(9600);
  delay(100);

  nexInit();
  delay(100);

  if (bme.begin()){Log.info("BME280 Sensor Initialized.");}
  else{Log.info("BME280 init failed");}

  Log.info("Setup Complete");

  ThingSpeak.begin(client);

  updateInterval = millis();
  min_last=Time.minute();
}

void loop() 
{
  //nexLoop(nex_listen_list);               //only need if triggering event from touchscreen.  Not needed to only display info
  if(Particle.disconnected()){return;}
  BME_HI=1000.0f;
  BME_DPF=1000.0f;

  if ((millis() - updateInterval) > UPDATE_INTERVAL)
  {
    timegrab();
    
    n000.setValue(hr_i);
    n001.setValue(min_i);
    t000.setText(ampm_s);
    t001.setText(weekday_s);
    t002.setText(month_s);
    n002.setValue(day_i);
     
    getBMEValues(BME_TC, BME_PPa, BME_RH);
    Log.info("Temperature(C): %f", BME_TC);
    Log.info("Pressure(Pa): %f", BME_PPa);
    Log.info("RelativeHumidity(%%): %f", BME_RH);

    BME_TF = (9.0f * BME_TC / 5.0f) + 32.0f;    // TF in Fahrenheit
    Log.info("Temperature(F): %f", BME_TF);

    BME_Ppsi = (BME_PPa / 6894.75729f);         // Pressure converted from Pa to psia
    Log.info("Pressure(psia): %f", BME_Ppsi);

    CalcHeatIndex(BME_TF, BME_RH, BME_HI);
    Log.info("HeatIndex(F): %f", BME_HI);

    CalcDewPoint(BME_TC, BME_RH, BME_DPF);
    Log.info("DewPoint(F): %f", BME_DPF);

    n100.setValue(int(BME_TF*10));      //formating for display with 0.1 precision
    n101.setValue(int(BME_Ppsi*1000));  //formating for display with 0.001 precision
    n102.setValue(int(BME_RH*10));      //formating for display with 0.1 precision
    n103.setValue(int(BME_HI*10));      //formating for display with 0.1 precision
    n104.setValue(int(BME_DPF*10));     //formating for display with 0.1 precision
   
    min_time=Time.minute();
    if((min_time!=min_last)&&(min_time==0||min_time==5||min_time==10||min_time==15||min_time==20||min_time==25
        ||min_time==30||min_time==35||min_time==40||min_time==45||min_time==50||min_time==55))
    {
      //createEventPayload2(MCP_TF, BME_TF, BME_Ppsi, BME_RH, BME_HI, BME_DPF);
    
      ThingSpeak.setField(1, BME_TF);
      ThingSpeak.setField(2, BME_Ppsi);
      ThingSpeak.setField(3, BME_RH);
      ThingSpeak.setField(4, BME_HI);
      ThingSpeak.setField(5, BME_DPF);
      ThingSpeak.writeFields(myTSChannel1, myTSWriteAPIKey1);
            
      min_last = min_time;
      Log.info("Last Update: %d", min_last);
      Log.info(Time.timeStr());
    }
            
    updateInterval = millis();
  }
}

void timegrab()
{
  Time.zone(-7);
  hr_i=Time.hourFormat12();
  min_i=Time.minute();
  ampm_i = Time.isPM();
  if(ampm_i==1){ampm_s=String("PM");}
  else{ampm_s=String("AM");}
  weekday_i=Time.weekday();
  if(weekday_i==1){weekday_s=String("SUN");}
  if(weekday_i==2){weekday_s=String("MON");}
  if(weekday_i==3){weekday_s=String("TUES");}
  if(weekday_i==4){weekday_s=String("WED");}
  if(weekday_i==5){weekday_s=String("THUR");}
  if(weekday_i==6){weekday_s=String("FRI");}
  if(weekday_i==7){weekday_s=String("SAT");}
  month_i=Time.month();
  if(month_i==1){month_s=String("JAN");}
  if(month_i==2){month_s=String("FEB");}
  if(month_i==3){month_s=String("MAR");}
  if(month_i==4){month_s=String("APR");}
  if(month_i==5){month_s=String("MAY");}
  if(month_i==6){month_s=String("JUNE");}
  if(month_i==7){month_s=String("JULY");}
  if(month_i==8){month_s=String("AUG");}
  if(month_i==9){month_s=String("SEPT");}
  if(month_i==10){month_s=String("OCT");}
  if(month_i==11){month_s=String("NOV");}
  if(month_i==12){month_s=String("DEC");}
  day_i=Time.day();
  
  Log.info("Hour: %d", hr_i);
  Log.info("Min: %d", min_i);
  Log.info("Is PM: %d", ampm_i);
  Log.info(ampm_s);
  Log.info("Weekday: %d", weekday_i);
  Log.info(weekday_s);
  Log.info("Month: %d", month_i);
  Log.info(month_s);
  Log.info("Day: %d", day_i);
  
  return;
}

float getBMEValues(float &BME_TC, float &BME_PPa, float &BME_RH)
{
  BME_TC = bme.readTemperature();
  //T in C
  BME_PPa = bme.readPressure();
  // Pressure in Pa
  BME_RH = bme.readHumidity();
  // RH in %
  return 1;
}

float CalcHeatIndex(float &BME_TF, float &BME_RH, float &BME_HI)
{   
  // Heat Index Calculations from nws
  // HIsimple if TF < 80
  if(BME_TF<=80)
  {
    float HIsimple = (BME_TF + 61.0f + (1.2f*(BME_TF-68.0f)) + (0.094*BME_RH)) / 2.0f;
    BME_HI = HIsimple;
  }
  // Rothfusz regression base equation
  float HIbasic = -42.379f + (2.04901523f*BME_TF) + (10.14333127f*BME_RH) -(0.22475541f*BME_TF*BME_RH) 
       - (0.00683783f*BME_TF*BME_TF) - (0.05481717f*BME_RH*BME_RH) + (0.00122874f*BME_TF*BME_TF*BME_RH) 
       + (0.00085282f*BME_TF*BME_RH*BME_RH) - (0.00000199f*BME_TF*BME_TF*BME_RH*BME_RH);

  //High humidity add correction RH> 85%, 80F<T<87;
  if(BME_TF>80.0f && BME_TF<=87.0f && BME_RH>=85.0f)
  {
    float HIwet = (BME_RH-85.0f)*(78.0f-BME_TF)/50.0f;
    BME_HI = HIbasic + HIwet;
  }
    
  //Low humidity correction RH<13, T from 80 to 112
  // float HIhigh = ((13.0f-RH)/4.0f)*sqrt((17-abs(TF-95))/17.0f)
  // break into cases to avoid undefined abs() which appears to only be for integers
  // float Hihigh1 = ((13.0f-RH)/4.0f)*sqrt((17-(95-TF))/17.0f) for 78F<T<95F
  if(BME_TF>80.0f && BME_TF<=95.0f && BME_RH<=13.0f)
  {
    float HIhigh1 = ((13.0f-BME_RH)/4.0f)*sqrt((17-(95-BME_TF))/17.0f);
    BME_HI = HIbasic - HIhigh1;
  }

  if(BME_TF>95.0f && BME_TF<=112.0f && BME_RH<=13.0f)
  {
    float HIhigh2 = ((13.0f-BME_RH)/4.0f)*sqrt((17-(BME_TF-95))/17.0f);
    BME_HI = HIbasic - HIhigh2;
  }
  else
  {
    BME_HI = HIbasic;
  }
  return BME_HI;
}

float CalcDewPoint(float &BME_TC, float &BME_RH, float &BME_DPF)
{
  //Dewpoint by Magus Equation with Sonntag 1990 constants
  // a=6.112 mbar, b=17.62, c=243.12 C with error < 0.1% for -45C to 60C (+/- 0.35C)
  float a = 6.112f;
  // in mbar
  float b = 17.62f;
  float c = 243.12f;
  float  gamma = log(BME_RH/100.0f) + (b*BME_TC/(c+BME_TC));
  float Pact = a*exp(gamma);
  // actual vapor pressure in mbar
  float BME_DPC = (c*log(Pact/a)) / (b - log(Pact/a));
  // dew point temperature in C
  BME_DPF = ((9.0f * BME_DPC) / 5.0f) + 32.0f;
  // dew point temperature in F
  return BME_DPF;
}

/*
void createEventPayload2(float MCP_TF, float BME_TF, float BME_Ppsi, float BME_RH, float BME_HI, float BME_DPF)
{
  JsonWriterStatic<256> jw;
  {
    JsonWriterAutoObject obj(&jw);
    jw.insertKeyValue("MHTempF",MCP_TF);
    jw.insertKeyValue("BMETempF",BME_TF);
    jw.insertKeyValue("Pressurepsia", BME_Ppsi);
    jw.insertKeyValue("Humidity%", BME_RH);
    jw.insertKeyValue("HeatIndexF", BME_HI);
    jw.insertKeyValue("DewPointF", BME_DPF);
  }
  Particle.publish("KobraPhysicals", jw.getBuffer(), PRIVATE);
}
*/