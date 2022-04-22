#include <EEPROM.h>
#include <LiquidCrystal_I2C.h>
#include <LM35.h>
#include <Wire.h>
#include "MAX30105.h"
#include "DHT.h"
#include "heartRate.h"
#include "pitches.h"
#include <BlynkSimpleSerialBLE.h>

#if defined(ARDUINO) && ARDUINO >= 100
#define printByte(args)  write(args);
#else
#define printByte(args)  print(args,BYTE);
#endif

#define BLYNK_AUTH_TOKEN            "6MfEtDlx9u8BbE08egKwpoEFPsZG54hU"
//#define BLYNK_PRINT Serial
#define REPORTING_PERIOD_MS 1000 // frequency of updates sent to blynk app in ms
#define TIMEOUT 30 //Time out second to sleep
//#define WAKEUP_SLEEP GPIO_NUM_33
#define TIMETOBOOT 3000 // wait for this time(msec) to output SpO2
#define SCALE 88.0 //adjust to display heart beat and SpO2 in the same scale
#define SAMPLING 1 //if you want to see heart beat more precisely , set SAMPLING to 1
#define FINGER_ON 30000 // if red signal is lower than this , it indicates your finger is not on the sensor
#define MINIMUM_SPO2 80.0
#define USEFIFO

#define Switcher 4
#define RESET 2
#define Redled 6
#define Greenled 7
#define Blueled 8
#define buzzer  9

//  ------------------------------------------------------------------------------
//  global Variables  global Variables  global Variables  global Variables  global
//  ------------------------------------------------------------------------------
char auth[] = BLYNK_AUTH_TOKEN;
byte lcdNumCols = 16; // -- number of columns in the LCD
byte lcdLine = 2; // -- number of line in the LCD
int barCols = 8;
int lastButtonState = 1;
long unsigned int lastPress;
int debounceTime = 20;
unsigned long previousMillis = 0;
unsigned long currentMillis;
unsigned long interval = 1000.0;
int view = 1;  int viewResult = 1;
int clicksCounter = 0;   // counter for the number of button presse

//  ------------------------------------------------------------------------------
//  Inits Object  Inits Object  Inits Object  Inits Object  Inits Object  Inits Ob
//  ------------------------------------------------------------------------------
LiquidCrystal_I2C lcd(0x27,lcdNumCols,lcdLine); //0x3f is address for I2C
BlynkTimer timer;
LM35 temp(A6);
MAX30105 particleSensor;
void initSpO2sensor()
{
  while (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    char errMsg="MAX30102 was not found. Please check wiring/power. ";
    Serial.println(errMsg);
    lcd.print(errMsg);
    //delay (1000);
  }
  //Setup to sense a nice looking saw tooth on the plotter
  byte ledBrightness = 0x7F; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  //Options: 1 = IR only, 2 = Red + IR on MH-ET LIVE MAX30102 board
  int sampleRate = 200; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 16384; //Options: 2048, 4096, 8192, 16384
  // Set up the wanted parameters
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
  particleSensor.enableDIETEMPRDY();
}
//  ------------------------------------------------------------------------------
//  bitmap  bitmap  bitmap  bitmap  bitmap  bitmap  bitmap  bitmap  bitmap  bitmap
//  ------------------------------------------------------------------------------
int prevValue = 0;
byte lastFullChars = 0;
byte first[8]   = { B10000, B10000, B10000, B10000, B10000, B10000, B10000, 16};
byte second[8]  = { B11000, B11000, B11000, B11000, B11000, B11000, B11000, 24};
byte third[8]   = { B11100, B11100, B11100, B11100, B11100, B11100, B11100, 28};
byte fourth[8]  = { B11110, B11110, B11110, B11110, B11110, B11110, B11110, 30};
byte full[8]    = { B11111, B11111, B11111, B11111, B11111, B11111, B11111, 31};
byte degree[8]  = {0b00011,0b00011,0b00000,0b00000,0b00000,0b00000,0b00000,0b00000};
byte smile[8]    = {B00000,B00000,B01010,B00000,B10001,B01110,B00000,B00000};
byte mod[8]      = {B00000,B00000,B01010,B00000,B11111,B00000,B00000,B00000};
byte sad[8]      = {B00000,B00000,B01010,B00000,B01110,B10001,B00000,B00000};
uint8_t heart[8]= {0x0,0xa,0x1f,0x1f,0xe,0x4,0x0};

//  ------------------------------------------------------------------------------
//  Heart Rate  Heart Rate  Heart Rate  Heart Rate  Heart Rate  Heart Rate  Heart 
//  ------------------------------------------------------------------------------
const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred
float beatsPerMinute;
int beatAvg;
float HB_ROM;
//  ------------------------------------------------------------------------------
//  EspO2  EspO2  EspO2  EspO2  EspO2  EspO2  EspO2  EspO2  EspO2  EspO2  EspO2
//  ------------------------------------------------------------------------------
int i = 0;
int32_t bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25sps,calculate SpO2 by this sampling interval
double avered = 0; double aveir = 0;
double sumirrms = 0;
double sumredrms = 0;
double ESpO2 = 95.0;//initial value of estimated SpO2
float ESpO2_ROM;
double FSpO2 = 0.7; //filter factor for estimated SpO2
double frate = 0.95; //low pass filter for IR/red LED value to eliminate AC component
float ir_forGraph;
float red_forGraph;
//  ------------------------------------------------------------------------------
//  temperature  temperature  temperature  temperature  temperature  temperature
//  ------------------------------------------------------------------------------
float Temperature;
float Temp_ROM;
//  ------------------------------------------------------------------------------
//  Buzzer Tone  Buzzer Tone  Buzzer Tone  Buzzer Tone  Buzzer Tone  Buzzer Tone  
//  ------------------------------------------------------------------------------
// notes in the melody:
int melody[] = {
  NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4
};
// beep sound
#define BOOTSOUND 440 //Hz
#define BLIPSOUND 440*2 //Hz A

// note durations: 4 = quarter note, 8 = eighth note, etc.:
int noteDurations[] = {  4, 8, 8, 4, 4, 4, 4, 4 };
//  ------------------------------------------------------------------------------
//  Blynk Functions  Blynk Functions  Blynk Functions  Blynk Functions  Blynk Func  
//  ------------------------------------------------------------------------------
// This function is called every time the Virtual Pin 0 state changes
BLYNK_WRITE(V0)
{
  // Set incoming value from pin V0 to a variable
  int value = param.asInt();

  // Update state
  Blynk.virtualWrite(V1, value);
}

// This function is called every time the device is connected to the Blynk.Cloud
BLYNK_CONNECTED()
{
  // Change Web Link Button message to "Congratulations!"
  Blynk.setProperty(V2, "url", "https://static-image.nyc3.cdn.digitaloceanspaces.com/general/fte/congratulations.png");
  Blynk.syncAll();
}
// This function sends Arduino's uptime every second to Virtual Pin 2.
void myTimerEvent()
{
  // You can send any value at any time.
  // Please don't send more that 10 values per second.
  Blynk.virtualWrite(V3, beatAvg);
  Blynk.virtualWrite(V4, ESpO2);
  Blynk.virtualWrite(V5, Temperature);
}

void setup()
{
  digitalWrite(RESET, HIGH);
  ESpO2 = readEEPROM();
  //Temperature = EEPROM.read(6);
  pinMode(Switcher, INPUT_PULLUP);
  pinMode(Greenled,OUTPUT);
  pinMode(Redled,OUTPUT);
  pinMode(Blueled,OUTPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(RESET, OUTPUT);   
  digitalWrite(Greenled,LOW);
  digitalWrite(Redled,LOW);
  Serial.begin(9600);
  int mytimeout = millis() / 1000;
  Blynk.begin(Serial,auth);
  /*while(!(millis() / 1000) > mytimeout + 8){
     Blynk.begin(Serial,auth);
  }*/
  timer.setInterval(1000L, myTimerEvent);
  Serial.println("Waiting for connections...");
  lcd.init();
  lcd.backlight();
  // Initialize sensor
  initSpO2sensor();
  flash_loading();
}


unsigned long timePress = 0;
unsigned long timePressLimit = 0;
int clicks = 0;
void loop()
{   
  if (Blynk.connected()) {    // If BLE is connected...
     Blynk.run(); 
     timer.run();
  }
  int buttonState = digitalRead(Switcher);
  if (buttonState != lastButtonState) {
    lcd.clear();
  }

  if(millis() - lastPress > debounceTime)
  {
    if(buttonState == 1 && lastButtonState ==0)
    {
      lastPress = millis();Serial.println("Button Pressed Once");lastButtonState = 1;
      lcd.clear();
      if (view < 3)       //Page counter never higher than 3 (total of pages)
        view++;                   //Go to next page
      else if ((view < 1) || view >= 3)
        view = 1;
    }
    else if(buttonState == 0 && lastButtonState == 1)//if button pressed and released last change
    {
      lastButtonState = 0;
      clicksCounter++;
      if(clicksCounter >= 2 && millis() - lastPress < interval/2){
        Serial.println("Button Pressed twice");     //Double Press Action
        for (int i = 0 ; i < EEPROM.length() ; i++) {
          EEPROM.write(i, 0);
        }
        digitalWrite(RESET, LOW);
        clicksCounter = 0;       
      }
    }
  }

  uint32_t ir, red , green;
  double fred, fir;
  double SpO2 = 0; //raw SpO2 before low pass filtered
  particleSensor.check(); //Check the sensor, read up to 3 samples
  while (particleSensor.available()) 
  {
    red = particleSensor.getFIFOIR();
    ir = particleSensor.getFIFORed(); 
    i++;    fred = (double)red;fir = (double)ir;
    avered = avered * frate + (double)red * (1.0 - frate);//average red level by low pass filter
    aveir = aveir * frate + (double)ir * (1.0 - frate); //average IR level by low pass filter
    sumredrms += (fred - avered) * (fred - avered); //square sum of alternate component of red level
    sumirrms += (fir - aveir) * (fir - aveir);//square sum of alternate component of IR level  
    heartrate(red);
    if ((i % SAMPLING) == 0) {//slow down graph plotting speed for arduino Serial plotter by thin out
      if ( millis() > TIMETOBOOT) {
        ir_forGraph = (2.0 * fir - aveir) / aveir * SCALE;
        red_forGraph = (2.0 * fred - avered) / avered * SCALE;
        //trancation for Serial plotter's autoscaling
        if ( ir_forGraph > 100.0) ir_forGraph = 100.0;
        if ( ir_forGraph < MINIMUM_SPO2) ir_forGraph = MINIMUM_SPO2;
        if ( red_forGraph > 100.0 ) red_forGraph = 100.0;
        if ( red_forGraph < MINIMUM_SPO2 ) red_forGraph = MINIMUM_SPO2;   
        if (ir < FINGER_ON){
          digitalWrite(Blueled, LOW);
          digitalWrite(Redled, LOW);
          digitalWrite(Greenled,LOW);
          ESpO2_ROM = ESpO2;
          writeEEPROM(&ESpO2_ROM);
          HB_ROM = beatAvg;
          writeEEPROM(&HB_ROM);
          //Temp_ROM = Temperature;
          //writeEEPROM(&Temp_ROM);
          
          if(ESpO2 == 0) {
            lcd.setCursor(0,1);lcd.print("Put your finger!");
            tone(buzzer,BOOTSOUND,8);
          }else {
            noTone(buzzer);
            lastResult(currentMillis);
          }   
          break;
        }        
        if(ir > FINGER_ON) 
        {
          updateView();
          if(ESpO2 < 70){
            digitalWrite(Blueled, HIGH);
            digitalWrite(Redled, LOW);
            digitalWrite(Greenled,LOW);
          }else if(ESpO2 >= 90 && Temperature < 37){
            digitalWrite(Blueled, LOW);
            digitalWrite(Redled,LOW);
            digitalWrite(Greenled,HIGH);
          }else if(((ESpO2 < 90) && (ESpO2 >= 70))|| Temperature >= 37){
            digitalWrite(Blueled, LOW);
            digitalWrite(Greenled,LOW);
            digitalWrite(Redled,HIGH);
          }
        }     
      }
    }
    if ((i % bufferLength) == 0) {
      double R = (sqrt(sumredrms) / avered) / (sqrt(sumirrms) / aveir);
      // Serial.println(R);
      SpO2 = -23.3 * (R - 0.4) + 100; //http://ww1.microchip.com/downloads/jp/AppNotes/00001525B_JP.pdf
      ESpO2 = FSpO2 * ESpO2 + (1.0 - FSpO2) * SpO2;//low pass filter
      //  Serial.print(SpO2);Serial.print(",");Serial.println(ESpO2);
      sumredrms = 0.0; sumirrms = 0.0; i = 0;
      break;
    }
    particleSensor.nextSample(); //We're finished with this sample so move to next sample
   //Serial.println(SpO2);   
  }
}

void writeEEPROM(float *data)
{
 byte ByteArray[4];
 memcpy(ByteArray, data, 4);
 for(int x = 0; x < 4; x++)
   EEPROM.write(x, ByteArray[x]);
}

float readEEPROM()
{
  float ESpO2 = 85.0;
  byte ByteArray[4];
  for(int x = 0; x < 4; x++)
    ByteArray[x] = EEPROM.read(x);    
  memcpy(&ESpO2, ByteArray, 4);
  return ESpO2;
}


void updateView() 
{
  unsigned long currentMillis = millis();
  switch (view) {
    case 1:
      tone(buzzer,BLIPSOUND - (100.0 - ESpO2) * 10.0,5); //when SpO2=80% BLIPSOUND drops 200Hz to indicate anormaly    
      lcd.setCursor(0,0);
      drawValue( ir_forGraph, 100);// -- draw bar graph from the analog value (ir/16384)
      lcd.setCursor(12,0);
      lcd.print("SpO2");
      lcd.setCursor(0,1);
      lcd.print("Oxy % = ");
      lcd.print(ESpO2);
      lcd.print(" % ");
      break;
    case 2:
      if(beatAvg != 0)
      {
        if (currentMillis - previousMillis >= 60) {
          // save the last time you blinked the LED
          previousMillis = currentMillis;
          tone(buzzer, (float)(rateSpot),(float)(60.0/beatAvg));
        }
      }else{
        noTone(buzzer);
      }
      lcd.setCursor(0,0);
      drawValue( red_forGraph, 100); // to display pulse wave at the same time with SpO2 data      
      lcd.setCursor(11,0);
      lcd.write(6);lcd.print("Beat");
      lcd.setCursor(0,1);
      lcd.print("BPM=");
      lcd.print(float(beatsPerMinute),1);
      lcd.print(" Avg=");
      lcd.print(beatAvg);
      lcd.write(6);
      break;
    case 3:
      if (currentMillis - previousMillis >= interval*2) {
        // save the last time you blinked the LED
        previousMillis = currentMillis;
        tone(buzzer, NOTE_B5,8);
        Temperature = temp.cel();      
        Temperature += 2;   
      }
      lcd.setCursor(0,0);
      lcd.print("Temperature: ");
      lcd.setCursor(4,1);
      lcd.print(Temperature);
      lcd.write(7);
      lcd.print("C");
      break;
  }
}

void lastResult(unsigned long currentMillis)
{
  for(int x = 0;x<6;x++)
  {
    switch(viewResult)
    {
      case 1:
      lcd.setCursor(0,0);
      lcd.print("Last test: ");
      lcd.setCursor(0,1);
      lcd.print("Oxy % = ");
      lcd.print(ESpO2_ROM);
      lcd.print(" % ");  
      break;
      case 2:
      lcd.setCursor(0,0);
      lcd.print("Average ");
      lcd.write(6);lcd.print("Beat : ");
      lcd.setCursor(8,1);
      lcd.print(HB_ROM);lcd.print(" ");
      lcd.write(6); 
      break;/*
      case 3:
      lcd.setCursor(0,0);
      lcd.print("Last Temperature: ");
      lcd.setCursor(4,1);
      lcd.print(Temp_ROM);
      lcd.write(7);
      lcd.print("C");
      break;*/
    }
    if (currentMillis - previousMillis >= interval) {
      // save the last time you blinked the LED
      previousMillis = currentMillis;
      lcd.clear();                 //lcd clear if page is changed.
      if (viewResult < 3)
      {                            //Page counter never higher than 3 (total of pages)
         viewResult++;                //Go to next page
      }
      else{
         viewResult=1;                //if counter higher than 3 (last page) return to page 1
      }
    }       
  }
}

void heartrate(uint32_t irValue)
{
  if (checkForBeat(irValue) == true)
  {
    //We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable

      //Take average of readings
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }
}

void drawChar()
{   
    lcd.createChar(1, first);
    lcd.createChar(2, second);lcd.createChar(3, third);
    lcd.createChar(4, fourth);lcd.createChar(5, full);
    
}

void drawValue(int value, int maxValue) 
{
    int range = map(value, 80, maxValue, 0, barCols);
    // -- calculate full (filled) character count
    byte fullChars = (long)value * barCols / maxValue;
    // -- calculate partial character bar count
    byte mod = ((long)value * barCols * 5 / maxValue) % 5;

    // -- if value does not change, do not draw anything
    int normalizedValue = (int)fullChars * 5 + mod;
    if(prevValue != normalizedValue) {
        // -- do not clear the display to eliminate flickers
        lcd.setCursor(0, 0);
        // -- write filled characters
        for(byte i=0; i<fullChars; i++) 
            lcd.write(5);  
        
        // -- write the partial character
        if(mod > 0) {
            lcd.write(mod); // -- index the right partial character
            ++fullChars;
        }
        
        // -- clear characters left over the previous draw
        for(byte i=fullChars;i<lastFullChars;i++) {
            lcd.write(' ');
        }
     
        // -- save cache
        lastFullChars = fullChars;
        prevValue = normalizedValue;
    }   
}

void flash_loading() {
  lcd.createChar(0, mod);lcd.createChar(1, smile);
  lcd.createChar(2, sad);lcd.createChar(6, heart);lcd.createChar(7, degree);
  bool warmUp=false;
  bool loading = false;
  bool plotSync = false;
  bool sound = false;
  loading = 1;
  plotSync = 1;
  sound = 1;
  while(plotSync)
  { 
    plotSync = 0;
    //Arduino plotter auto-scales annoyingly. To get around this, pre-populate
    //the plotter with 500 of an average reading from the sensor
    //Take an average of IR readings at power up
    long baseValue = 0;
    const byte avgAmount = 64;
    for (byte x = 0 ; x < avgAmount; x++)
    {
      baseValue += particleSensor.getIR(); //Read the IR value
    }
    baseValue /= avgAmount;
    if(loading)
    {
      loading = 0;
      int no = 0;
      lcd.setCursor(0,0);
      lcd.print(F("WarmUP:"));
      //PRINTING LOADING BAR AND THE PERCENTAGE
      float value=0;
      for (int j=0; j<=15; ++j) {
        lcd.setCursor(j,1);
        for (int i=1; i<=5; ++i) {
          for (int x = 0 ; x <= 500/75; x++)
          {//Pre-populate the plotter so that the Y scale is close to IR values
            Serial.println(baseValue);
          }
          lcd.setCursor(j,1);
          if(j%2 != 0) 
            lcd.write(6);
          else lcd.write(j%3);;
          lcd.setCursor(10,0);
          value+=(((float)5)/4);
          lcd.print(value);
    
          if (value>=99.99) {
            lcd.setCursor(15,0);
          }
          lcd.print("%");
        }
      } 
      if(sound)
      {
        // declaring variables
        unsigned long previousMillis = 0;
        const int MelodyLength = 8;
        bool outputTone = false;
        sound = 0;
        int thisNote = 0;
        if(!sound)
        {
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("  Tino's Pulse");
          lcd.setCursor(0, 1);
          lcd.print("    Oximeter");
          drawChar();
          while(thisNote<=MelodyLength)
          {
            unsigned long currentMillis= millis();
            int noteDuration = 1000 / noteDurations[thisNote];
            int pauseBetweenNotes = noteDuration * 0.30;
            if (outputTone) {
              // We are currently outputting a tone
              // Check if it's been long enough and turn off if so
              if (currentMillis - previousMillis >= noteDuration) {
                previousMillis = currentMillis;
                noTone(buzzer);
                outputTone = false;
              }
            } else {
              // We are currently in a pause
              // Check if it's been long enough and turn on if so
              if (currentMillis - previousMillis >= pauseBetweenNotes) {
                previousMillis = currentMillis;
                tone(buzzer, melody[thisNote], noteDuration);//tone(tonePin, Melody[MelodyIndex]);
                outputTone = true;
                //Update to play the next tone, next time
                thisNote++;
                if (thisNote >= MelodyLength) {
                  break;
                }
              }
            }
          }
          lcd.scrollDisplayLeft();
        } 
      }
    }
  }
  lcd.noAutoscroll();
  lcd.clear();
}
