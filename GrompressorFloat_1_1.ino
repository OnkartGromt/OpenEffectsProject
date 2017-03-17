#include <Adafruit_GFX.h>

/*
Grompressor1.ino
Compressor effect using floating point core. Thanks to Chip Audette! 
Thanks to Paul Stoffregen, www.PJRC.com, that made the great audio library fro Teensy! Plus all the other great stuff!
This code can be loaded into a Teensy3.5/3.6 with a OnkartGromt OpenEffects project platform. Use a I2C 64x128 OLED screen to show the graphics. 
Hold the On/Off switch for 1 sek to edit the input gain with pot1. 
All in all use the OpenEffectsBox ref2.2 from OnkartGromt with Teensy 3.5/3.6
 */
#include <EEPROM.h> 
#include <Bounce.h> 
#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>
#include<string.h>
#include <OpenAudio_ArduinoLibrary.h> //for AudioConvert_I16toF32, AudioConvert_F32toI16, and AudioEffectGain_F32
using namespace std;

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);
//Blue HW:
#define PIN            8
//RED HW:
//#define PIN            3

#define NUMPIXELS      10
#define bands          6
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

// GUItool: begin automatically generated code
AudioInputI2S        i2sAudioIn1;    //xy=144,485
AudioConvert_I16toF32    int2Float2;     //xy=317,425
AudioConvert_I16toF32    int2Float1;     //xy=342,549
AudioEffectCompressor_F32 comp1;          //xy=510,549
AudioEffectCompressor_F32 comp2;          //xy=511,425
AudioConvert_F32toI16    float2Int2;     //xy=654,425
AudioConvert_F32toI16    float2Int1;     //xy=670,549
AudioMixer4          mixer1;       //xy=847,444
AudioMixer4          mixer2;       //xy=851,542
AudioAnalyzeNoteFrequency notefreq1;
AudioOutputI2S       i2sAudioOut1;   //xy=1022,482
AudioConnection         patchCord1(i2sAudioIn1, 0, int2Float2, 0);
AudioConnection         patchCord2(i2sAudioIn1, 0, mixer1, 1);
AudioConnection         patchCord3(i2sAudioIn1, 1, int2Float1, 0);
AudioConnection         patchCord4(i2sAudioIn1, 1, mixer2, 1);
AudioConnection_F32         patchCord5(int2Float2, comp2);
AudioConnection_F32         patchCord6(int2Float1, comp1);
AudioConnection_F32         patchCord7(comp1, float2Int1);
AudioConnection_F32         patchCord8(comp2, float2Int2);
AudioConnection         patchCord9(float2Int2, 0, mixer1, 0);
AudioConnection         patchCord10(float2Int1, 0, mixer2, 0);
AudioConnection         patchCord11(mixer1, 0, i2sAudioOut1, 0);
AudioConnection         patchCord12(mixer2, 0, i2sAudioOut1, 1);
AudioControlSGTL5000     sgtl5000;     //xy=385,640
// GUItool: end automatically generated code
AudioAnalyzeRMS          rms1, rms2, rms3, rms4;  //peak indicator/detection
AudioConnection         patchCord22(i2sAudioIn1, 0, rms2, 0);
AudioConnection         patchCord23(i2sAudioIn1, 1, rms1, 0);
AudioConnection         patchCord24(float2Int1, 0, rms3, 0);
AudioConnection         patchCord25(float2Int1, 0, rms4, 0);
AudioConnection         patchCord26(float2Int1, 0, notefreq1, 0);

//Pinout board rev1
int Pot1 = A6;    // select the input pin for the potentiometer
int Pot2 = A3;    // select the input pin for the potentiometer
int Pot3 = A2;    // select the input pin for the potentiometer
int Pot4 = A1;    // select the input pin for the potentiometer
int CV1 = A10;
int CV2 = A11;

//For the RED pcb
//int Tap1 = 0; 
//int Tap2 = 1;

//For Blue/black PCB:
int Tap1 = 2;
int Tap2 = 3;

int SW1 = A12;
int SW2 = A13;

//int ledPin1 = 3;      // select the pin for the LED strip
int relayL = 4;      // select the pin for the RelayL
int relayR = 5;      // select the pin for the RelayR

//Variables for values
boolean Tap1Value = 0;
boolean Tap2Value = 0;
float Pot1Value = 0;  
float Pot2Value = 0;  
float Pot3Value = 0;  
float Pot4Value = 0;  
int SW1Value = 0;
int SW2Value = 0;
float CV1Value = 0;
float CV2Value = 0;
float Pot1ValueOld = 0;  // variable to store the previous value
float Pot2ValueOld = 0;  
float Pot3ValueOld = 0;  
float Pot4ValueOld = 0;  
int SW1ValueOld = 0;
int SW2ValueOld = 0;
int Tap1ValueOld = 1;
int Tap2ValueOld = 1;
int CV1ValueOld = 0;
int CV2ValueOld = 0;
int Tap1Timer = 0;
int Tap2Timer = 0;
float margin  = 0.1;
//int modulationMode = 0;
boolean Tap2Pressed = 0;
//int filterOrder = 24;
//float ffund = 0;

// Variables 
int ledStateONOFF = LOW;         // the current state of the output pin
int ledStateONOFF_old = LOW;
int buttonState0;             // the current reading from the input pin
int lastButtonState0 = LOW;   // the previous reading from the input pin
int ledStateBOOST = LOW;         // the current state of the output pin
int buttonState1;             // the current reading from the input pin
int lastButtonState1 = LOW;   // the previous reading from the input pin

unsigned long lastDebounceTime0 = 0;  // the last time the output pin was toggled
unsigned long lastDebounceTime1 = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 40;    // the debounce time; increase if the output flickers

float peak = 0.0;
float peak_2 = 0.0;
float peak_3 =0.0;
float peak_4 =0.0;
float level = 1;
float InLevel = 1;
float outLevel = 1;
float inLevel = 0.05;
float bypassLevel = 0.42;
float mixer3Level = 1.0;
int lastParam = 0;

int updateParam = 0;
float maxGain = 0;
float response = 0;
float thres = -18;
float hardLimit = 0;
float attack_1 = 0.01f;
float decay_1 = 0.2f;
int Ratio = 3;
int inlevel = 5;

struct presets {
  float thres;
  float attack_1;
  float decay_1;
  int Ratio;
  int inputGain;
  int actualPreset;
}  ;
//new version:

float levelBand[bands] = {30,35, 40, 25, 50,60};
float levelBandScaled[bands] = {30,35, 40, 25, 50,60};
float levelBanddB[bands] = {0,0,0, 0, 0,0};
unsigned int freqBand[bands] = {30, 160, 320, 640, 2000,16000};
float freqBandDefault[bands] = {80, 160, 320, 640, 2000,16000};
float freqBandScaled[bands] = {4, 12, 32, 64, 110,110};
float slopeBand[bands] = {.7,.7,.7,.7,.7,.7};
float Qlevel[bands] = {1,1,1,1,1,1};
int updateFilter = 1;
int savePreset = 0;
int loadPreset = 0;
int actualPreset = 1;
int numberOfPresets = 10;
unsigned char loByte;
unsigned char hiByte;
uint32_t off =pixels.Color(0,0,0);
uint32_t blue =pixels.Color(0,0,50);
uint32_t red =pixels.Color(0,50,0);
uint32_t green =pixels.Color(0,40,0);
uint32_t yellow =pixels.Color(40,40,0);
int VUlag = 0;
const int myInput = AUDIO_INPUT_LINEIN;

Bounce OnOff = Bounce(Tap1, 30 ); 
Bounce BoostOnOff = Bounce(Tap2, 30); 
elapsedMillis holdTime;
elapsedMillis tapInterval;  //timer that is used for tap tempo

void setup() {
  AudioMemory(60);
  AudioMemory_F32(40); //allocate Float32 audio data blocks
  pixels.begin();
  for (int i = 0;i<NUMPIXELS;i++){
    pixels.setPixelColor(i,off);
   } 
  pixels.setPixelColor(0,green);
  pixels.show();
  InLevel = float(EEPROM.read(0)/255.0);
  outLevel = float(EEPROM.read(1));
  sgtl5000.enable();  // Enable the audio shield
  sgtl5000.inputSelect(myInput);
  sgtl5000.volume(0.8);
  sgtl5000.lineInLevel(15);   // 3.12Vp-p
  sgtl5000.lineOutLevel(21); // 3.16Vp-p
  sgtl5000.adcHighPassFilterDisable(); 
  //READ PRESET FROM EEPROM
  if (EEPROM.read(10) == 0) EEPROM.write(10,1);
  
  //pinMode(ledPin1, OUTPUT); ONLY WITH RED BOARD
  pinMode(relayL, OUTPUT);
  pinMode(relayR, OUTPUT);
  pinMode(Tap1, INPUT);
  pinMode(Tap2, INPUT);
 
  mixer1.gain(0, 1);
  mixer1.gain(1, 0);
  mixer1.gain(2, 0);  
  mixer1.gain(3, 0);  
  mixer2.gain(0, 1);
  mixer2.gain(1, 0);
  mixer2.gain(2, 0);  
  mixer2.gain(3, 0);  
  
  sgtl5000.inputSelect(myInput);
  sgtl5000.dacVolume (level);
  sgtl5000.autoVolumeControl(0,0,0,0.7,50,100);  //maxGain,response,hard limit,threshold,attack, decay
  sgtl5000.autoVolumeDisable();
  sgtl5000.audioPostProcessorEnable();
  sgtl5000.eqSelect(3);
  sgtl5000.eqBands(0,0,0,-1.0,-1.0);
  sgtl5000.dacVolumeRamp();
  Serial.begin(115200);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x64)
  display.clearDisplay();
  pixels.show();
  delay(100);
  Serial.println("setup complete");
}

//Main loop
void loop() {
  OnOff.update();      //update the bounce
  BoostOnOff.update();

  // read the value from the pots and buttons:
  Pot1Value = analogRead(Pot1);    //Level
  Pot2Value = analogRead(Pot2);    //Width
  Pot3Value = analogRead(Pot3);    //Freq
  Pot4Value = analogRead(Pot4);    //Mode
  Tap1Value = digitalRead(Tap1);   //On off
  Tap2Value = digitalRead(Tap2);   //Mode/Tap
  SW1Value = analogRead(SW1);      //Filter char
  SW2Value = analogRead(SW2);      //Direction

  SW1Value = map(SW1Value, 0, 1023, 5, 0);               //Filter char
  SW1Value = constrain(SW1Value, 1, 5);
  SW2Value = map(SW2Value, 0, 1023, 5, 0);               //Direction
  SW2Value = constrain(SW2Value, 1, 5);

//********************ON -OFF handling**********************
  buttonState0 = OnOff.read();
  if (lastButtonState0 != buttonState0){   
      // only toggle the state if the new button state is HIGH
      if (buttonState0 == LOW) {
        ledStateONOFF = !ledStateONOFF;
      }
  }
  buttonState1 = BoostOnOff.read();
  if (lastButtonState1 != buttonState1){ 
      // only toggle the state if the new button state is HIGH
      if (buttonState1 == LOW) {
        ledStateBOOST = !ledStateBOOST;
      }
  } 
  // set the outputs:
  if (ledStateONOFF != ledStateONOFF_old){  
    pixels.setPixelColor(2, pixels.Color(ledStateONOFF*30,(ledStateONOFF*30),ledStateONOFF*30)); // ONOFF bright white color. 
    digitalWrite(relayL, ledStateONOFF); 
    digitalWrite(relayR, ledStateONOFF);
    ledStateONOFF_old = ledStateONOFF; 
    pixels.show(); // This sends the updated pixel color to the hardware.  
  }
  lastButtonState0 = buttonState0;  //save readings to next round
  lastButtonState1 = buttonState1;
  
//********************PRESET handling**********************
//
if (SW2Value == 1){
  actualPreset--;
  if (actualPreset <0)actualPreset = numberOfPresets;
  delay(50);
  }
if (SW2Value == 5){
  actualPreset++;
  if (actualPreset > numberOfPresets)actualPreset =0;
  delay(50);
  }
    
//******************** Handle user inputs **********************  

if (SW1Value == 1){
    Ratio ++;
    if (Ratio >=20)Ratio = 20;
    updateParam = 1;
  }
if (SW1Value == 5){
   Ratio --;
   if (Ratio <=0)Ratio = 1;
   updateParam = 1;
  }
int margin = 3;
  
if ((Pot1Value > Pot1ValueOld+ margin) || (Pot1Value < Pot1ValueOld - margin)){  //LEVEL adjust
    inlevel = map(Pot1Value, 0, 1023, 15, 0); 
    sgtl5000.lineInLevel(inlevel);   // 3.12Vp-p
    updateParam = 1;
}

if ((Pot2Value > Pot2ValueOld+ margin) || (Pot2Value < Pot2ValueOld - margin)){  //Threshold values
    thres = map(Pot2Value, 0, 1023, 0,-70.0f); 
    updateParam = 1;
}

if ((Pot3Value > Pot3ValueOld+ margin) || (Pot3Value < Pot3ValueOld - margin)) {    //Attack
    attack_1 = mapfloat(Pot3Value, 0, 1023, 0.2f,0.001f); 
      Serial.println(attack_1);
    
    updateParam = 1;
}

if ((Pot4Value > Pot4ValueOld+ margin) || (Pot4Value < Pot4ValueOld - margin)){  //Release
    decay_1 = mapfloat(Pot4Value, 0, 1023,2.0, 0.02); 
    updateParam = 1; 
          Serial.println(decay_1);
}
if (updateParam == 1){
    //configure the left and right compressors with the new settings
    boolean use_HP_filter = true; //enable the software HP filter to get rid of DC
    setupMyCompressors(use_HP_filter, thres, Ratio, attack_1, decay_1);
    updateParam = 0;
    }

//********************Load and saving presets **********************
if (BoostOnOff.fallingEdge()){
  holdTime = 0;
  Serial.println("pushed");
 }
if (!BoostOnOff.read() && holdTime >1500) {
        savePreset = 1;  
        pixels.setPixelColor(1, green);    
        pixels.show();
        Serial.println(holdTime);
        Serial.println("Save");

  }
else if (BoostOnOff.risingEdge() && (holdTime >600) && (holdTime <1500)) {
        actualPreset++;  
        if (actualPreset >=10)actualPreset = 0;
        loadPreset = 1;
        pixels.setPixelColor(1, red);    
        pixels.show();
        Serial.print("Loading new Preset: ");
        Serial.println(actualPreset);
        delay(500);
  }     
else if (BoostOnOff.risingEdge()){
        loadPreset = 1;
        Serial.print("Load");
        Serial.println(holdTime);
   }
   
      
if (savePreset == 1){
    /*
    uint16_t value = (uint16_t)(levelBand[i]*100);
    loByte = uint8_t(value & 0x00ff);
    hiByte = uint8_t((value >> 8 )& 0x00ff);     
    EEPROM.write(20+2*i+actualPreset*70,(loByte));
    EEPROM.write(20+1+2*i+actualPreset*70,(hiByte));
    value = (uint16_t)(slopeBand[i]*100);    
    loByte = uint8_t(value & 0x00ff);
    hiByte = uint8_t((value >> 8 )& 0x00ff);  
    EEPROM.write(40+2*i+actualPreset*70,(loByte)); 
    EEPROM.write(40+1+2*i+actualPreset*70,(hiByte));    
    loByte = uint8_t(freqBand[i] & 0x00ff);
    hiByte = uint8_t((freqBand[i] >> 8 )& 0x00ff);     
    EEPROM.write(60+2*i+actualPreset*70,loByte);
    EEPROM.write(60+1+2*i+actualPreset*70,hiByte);
    */
printParameters(&Serial);  
    
    
 Serial.print("Saved");
 Serial.println(actualPreset); 
    display.setCursor(68, 10);
    display.print("SAVED");
    display.display();
    delay(1000);
 savePreset = 0;
 } 

if (loadPreset ==1){

    /*
    unsigned int hg[i];
    loByte= EEPROM.read(20+2*i+actualPreset*70);
    hiByte= EEPROM.read(20+1+2*i+actualPreset*70);
    levelBand[i] = (float)((hiByte << 8) | loByte)/100; 
    loByte= EEPROM.read(40+2*i+actualPreset*70);    
    hiByte= EEPROM.read(40+1+2*i+actualPreset*70);
    slopeBand[i] = (float)((hiByte << 8) | loByte)/100;     
    loByte = EEPROM.read(60+2*i+actualPreset*70); 
    hiByte = EEPROM.read(60+1+i*2+actualPreset*70);     
    freqBand[i] = (hiByte << 8) | loByte;
    */
 
    printParameters(&Serial);
  
  Serial.print("loading..");
  Serial.println(actualPreset);
  loadPreset = 0;
  updateParam=1;
    display.setCursor(68, 10);
    display.print("LOAD");
    display.display();
    delay(500);
 } 
display.clearDisplay();

//*******************INPUT/OUTPUT LEVEL ADJUST************************
if (SW1Value == 5){    //LP pos for input level
 if (!Tap1Value && (Tap1Value == Tap1ValueOld)){    //When Mode/tap is held down you can adjust the input gain of the input
 // Serial.print("TIME");
   Tap1Timer++;
  if (Tap1Timer > 15){
    InLevel = analogRead(Pot1);
    InLevel = mapfloat(InLevel,0,1023,10,1);
    InLevel = InLevel * inLevel; //multiply by constant inLevel
    delay(100); 
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(5, 10);
    display.print("Input:");
    display.print(InLevel); 
    display.display();
    }
  }
  else if (Tap1Timer > 15){    //only write to EEPROM once
    EEPROM.write(0, InLevel*255);
  //  Serial.println(InLevel);
    Tap1Timer = 0;
  }
  else if (Tap1Value){
  //  peak = peak1.read();   //read the signal output peak
  if (peak == -1)(peak = 0);
  //analogWrite(ledPin2, peak*100);
  //pixels.setPixelColor(1, pixels.Color(peak*100,peak*100,0));
  //pixels.show();
  }
}

if (SW1Value == 1){    //HP pos for Output level
 if (!Tap1Value && (Tap1Value == Tap1ValueOld)){    //When Mode/tap is held down you can adjust the input gain of the input
 // Serial.print("TIME");
   Tap1Timer++;
  if (Tap1Timer > 15){
    outLevel = analogRead(Pot1);
    outLevel = mapfloat(outLevel,0,1023,10,1);
    delay(100);
   // analogWrite(ledPin2, 255); 
    //pixels.setPixelColor(1, pixels.Color(70,70,0));
    //display.clearDisplay();   
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(5, 10);
    display.print("Output:");
    display.print(outLevel); 
    display.display();
    }
  }
  else if (Tap1Timer > 15){    //only write to EEPROM once
    EEPROM.write(1, outLevel);
    Tap1Timer = 0;
  }
  else if (Tap1Value){
  //  peak = peak1.read();   //read the signal output peak
  if (peak == -1)(peak = 0);
  //analogWrite(ledPin2, peak*100);
  //pixels.setPixelColor(1, pixels.Color((peak)*100,(peak)*100,0));
  //pixels.show();
  }
}

 ////// VU meter ////////////
//Gain reduction meter

for(int i=NUMPIXELS;i>=3;i--){
  float reduction = abs(comp1.getCurrentGain_dB());
  int lvl = NUMPIXELS-(reduction/5);       //120*7=840
  int grade = 360+reduction*960;
  //Serial.println(peak_2);
  grade = grade -(120*lvl);

  if ((i > lvl)&&(i == 5))pixels.setPixelColor(i,yellow); // yellow color.
  if ((i > lvl)&&(i == 4))pixels.setPixelColor(i,red); // red color.
  else if (i > lvl)pixels.setPixelColor(i,green); // green color.
  //else if (i == lvl){
 //   pixels.setPixelColor(i, pixels.Color(grade,0,0)); // green color.
  //}
  else     {
    pixels.setPixelColor(i,off); // no color
  }
}
  pixels.show(); // This sends the updated pixel color to the hardware.  

///////////end VU meter/////////

///////////////////////////PLOTTING OF COMPRESSOR CURVE//////////////////////////////////
//Draw background:
int top = 20;
int signalWidth = 6;
display.drawLine(0,top,128,top, WHITE);
display.drawLine(0,64,0,top, WHITE);
display.drawLine(signalWidth+1,64,signalWidth+1,top, WHITE);
display.drawLine(signalWidth*2+2,64,signalWidth*2+2,top, WHITE);
display.drawLine(127-signalWidth*2-2,64,127-signalWidth*2-2,top, WHITE);
display.drawLine(127-signalWidth-1,64,127-signalWidth-1,top, WHITE);
display.drawLine(127,64,127,top, WHITE);

int thresh_disp = map(thres,0,-70,(64-top),10);
int zeroX = signalWidth*2+8;
int diffY = (64-thresh_disp) - top;
int kneeX = zeroX+thresh_disp;
int kneeY = 64-thresh_disp;
int Xtop = kneeX + diffY*Ratio;

display.drawLine(zeroX,64, kneeX, kneeY, WHITE);   //first slope
display.drawLine(kneeX, kneeY, Xtop,top, WHITE);   //second slope
display.fillRect(127-signalWidth*2-1,top+1,signalWidth,(64-top),BLACK);
display.fillRect(127-signalWidth,top+1,signalWidth,(64-top),BLACK);

  //read the signal input peak
if (rms1.available()){
  peak = rms1.read();
  float peakDisp1 = top+((64-top)*(1-peak));
  display.fillRect(1,peakDisp1,signalWidth,(64),WHITE);
  }
  
if (rms2.available()) {
  peak_2 = rms2.read();    //Output peak
  float peakDisp2 = top+((64-top)*(1-peak_2));
  display.fillRect(2+signalWidth,peakDisp2,signalWidth,(64),WHITE);
  }
//draw gain reduction
 // (comp2.getCurrentGain_dB());
display.fillRect(127-signalWidth*2-1,top+1,signalWidth,abs(comp1.getCurrentGain_dB()),WHITE);
display.fillRect(127-signalWidth,top+1,signalWidth,abs(comp2.getCurrentGain_dB()),WHITE);
 /////////////////////END PLOTTING///////////////////////////////
 
 ////// VU meter ////////////
 /*
VUlag++;
if (VUlag >0){
for(int i=3;i<=NUMPIXELS;i++){
  int lvl = 3+(peak_2*12);       //120*7=840
  int grade = 360+peak_2*960;
  //Serial.println(peak_2);
  grade = grade -(120*lvl);
  if (i < lvl){
    pixels.setPixelColor(i, pixels.Color(80,0,0)); // green color.
  }
  else if (i == lvl){
    pixels.setPixelColor(i, pixels.Color(grade,0,0)); // green color.
  }
  else     {
    pixels.setPixelColor(i, pixels.Color(0,0,0)); // no color
  }
}
  pixels.show(); // This sends the updated pixel color to the hardware.  
 // VUlag = 0;
}
*/
///////////end VU meter/////////
 
 // int cpuCap = AudioProcessorUsageMax();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(10, 10);

  display.print("Th:");
  display.print((int)thres);  
  display.print("dB");  
  
  display.setCursor(17, 23);
  display.print(Ratio); 
  display.print(":1"); 
  
  display.setCursor(100, 10);
  display.print("P:");
  display.print(actualPreset);   
  
  display.setCursor(30, 56);
  display.print("A:");
  display.print((int)(attack_1*1000));  
  display.print("ms");
  display.setCursor(70, 56);
  display.print(" R:");
  display.print((int)(decay_1*1000));    
  display.display();
  
 /*
   Serial.print(" Last: ");
   Serial.print(Pot1ValueOld);
   Serial.print(" Last: ");
   Serial.println(Pot1Value);
  
   Serial.print(" Freq2: ");
   Serial.print(freqBand[2]);  
   Serial.print(" Freq3: ");
   Serial.print(freqBand[3]); 
   Serial.print(" Freq4: ");
   Serial.print(freqBand[4]); 
   Serial.print(" Freq5: ");
   Serial.println(freqBand[5]);   
   Serial.print(" Peak2: ");
   Serial.println(peak_2);   
   */
   Pot1ValueOld = Pot1Value;
   Pot2ValueOld = Pot2Value;   
   Pot3ValueOld = Pot3Value;
   Pot4ValueOld = Pot4Value; 
   SW1ValueOld = SW1Value;
   SW2ValueOld = SW2Value;
   Tap1ValueOld = Tap1Value;
   Tap2ValueOld = Tap2Value;   
   CV1ValueOld = CV1Value;
   CV2ValueOld = CV2Value;
//   modFreqOld = modFreq;
//   spanfreqOld = spanfreq;
//   movingfreqOld = movingfreq;

    //printCPUandMemoryUsage(&Serial);
   //AudioInterrupts();
   //AudioNoInterrupts();
 //  pixels.show();
//printCompressorState(&Serial);
/*  if (notefreq1.available()){
  Serial.println(notefreq1.read());
 }
 */
}

void printCompressorState(Stream *s) {
  s->print("Current Compressor: Pre-Gain (dB) = ");
  s->print(comp1.getPreGain_dB());
  s->print(", Level (dBFS) = ");
  s->print(comp1.getCurrentLevel_dBFS());
  s->print(", ");
  s->print(comp2.getCurrentLevel_dBFS());
  s->print(", Dynamic Gain L/R (dB) = ");
  s->print(comp1.getCurrentGain_dB());
  s->print(", ");
  s->print(comp2.getCurrentGain_dB());
  s->println();
};

void printParameters(Stream *s) {
    s->print("Preset: ");
    s->print(actualPreset);
    s->print(" InputGain: ");
    s->print(inLevel); 
    s->print(" Treshold: ");
    s->print(thres);
    s->print(" Ratio: 1: ");
    s->print(Ratio);     
    s->print(" Attack: ");
    s->print(attack_1);
    s->print(" Release: ");
    s->println(decay_1); 
};
/* fscale
 Floating Point Autoscale Function V0.1
 Paul Badger 2007
 Modified from code by Greg Shakar

 This function will scale one set of floating point numbers (range) to another set of floating point numbers (range)
 It has a "curve" parameter so that it can be made to favor either the end of the output. (Logarithmic mapping)
*/
float fscale( float originalMin, float originalMax, float newBegin, float
newEnd, float inputValue, float curve){

  float OriginalRange = 0;
  float NewRange = 0;
  float zeroRefCurVal = 0;
  float normalizedCurVal = 0;
  float rangedValue = 0;
  boolean invFlag = 0;


  // condition curve parameter
  // limit range

  if (curve > 10) curve = 10;
  if (curve < -10) curve = -10;

  curve = (curve * -.1) ; // - invert and scale - this seems more intuitive - postive numbers give more weight to high end on output 
  curve = pow(10, curve); // convert linear scale into lograthimic exponent for other pow function

  /*
   Serial.println(curve * 100, DEC);   // multply by 100 to preserve resolution  
   Serial.println(); 
   */

  // Check for out of range inputValues
  if (inputValue < originalMin) {
    inputValue = originalMin;
  }
  if (inputValue > originalMax) {
    inputValue = originalMax;
  }

  // Zero Refference the values
  OriginalRange = originalMax - originalMin;

  if (newEnd > newBegin){ 
    NewRange = newEnd - newBegin;
  }
  else
  {
    NewRange = newBegin - newEnd; 
    invFlag = 1;
  }

  zeroRefCurVal = inputValue - originalMin;
  normalizedCurVal  =  zeroRefCurVal / OriginalRange;   // normalize to 0 - 1 float

  /*
  Serial.print(OriginalRange, DEC);  
   Serial.print("   ");  
   Serial.print(NewRange, DEC);  
   Serial.print("   ");  
   Serial.println(zeroRefCurVal, DEC);  
   Serial.println();  
   */

  // Check for originalMin > originalMax  - the math for all other cases i.e. negative numbers seems to work out fine 
  if (originalMin > originalMax ) {
    return 0;
  }

  if (invFlag == 0){
    rangedValue =  (pow(normalizedCurVal, curve) * NewRange) + newBegin;

  }
  else     // invert the ranges
  {   
    rangedValue =  newBegin - (pow(normalizedCurVal, curve) * NewRange); 
  }

  return rangedValue;
}

//************EEPROM functions****************
//This function will write a 2 byte (16bit) uint16_t to the eeprom at
//the specified address to address + 1.
void EEPROMWrite_uint16_t(int address, uint16_t value)
      {
      //Decomposition from a uint16_t to 2 bytes by using bitshift.
      //One = Most significant -> Two = Least significant byte
      byte two = ((value >> 0xFF) & 0xFF);
      byte one = ((value >> 8) & 0xFF);

      //Write the 2 bytes into the eeprom memory.
      EEPROM.write(address, two);
      EEPROM.write(address + 1, one);
      }
  
long EEPROMRead_uint16_t(int address)
      {
      //Read the 2 bytes from the eeprom memory.

      uint16_t two = EEPROM.read(address);
      uint16_t one = EEPROM.read(address + 1);

      //Return the recomposed uint16_t by using bitshift.
      return ((two << 16) & 0xFF) + ((one << 24) & 0xFFFF);
      }      


void printCPUandMemoryUsage(Stream *s) {
  s->print("Usage/Max: ");
  s->print("comp1 CPU = "); s->print(comp1.processorUsage()); s->print("/"); s->print(comp1.processorUsageMax()); s->print(", ");
  s->print("all CPU = " ); s->print(AudioProcessorUsage()); s->print("/");  s->print(AudioProcessorUsageMax()); s->print(", ");
  s->print("Int16 Mem = "); s->print(AudioMemoryUsage()); s->print("/"); s->print(AudioMemoryUsageMax()); s->print(", ");
  s->print("Float Mem = "); s->print(AudioMemoryUsage_F32()); s->print("/"); s->print(AudioMemoryUsageMax_F32()); s->print(", ");
  s->println();
};      

/////////////////////////////////////////

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//define a function to configure the left and right compressors
void setupMyCompressors(boolean use_HP_filter, float knee_dBFS, float comp_ratio, float attack_sec, float release_sec) {
  comp1.enableHPFilter(use_HP_filter);   comp2.enableHPFilter(use_HP_filter);
  comp1.setThresh_dBFS(knee_dBFS);       comp2.setThresh_dBFS(knee_dBFS);
  comp1.setCompressionRatio(comp_ratio); comp2.setCompressionRatio(comp_ratio);

  float fs_Hz = AUDIO_SAMPLE_RATE;
  comp1.setAttack_sec(attack_sec, fs_Hz);       comp2.setAttack_sec(attack_sec, fs_Hz);
  comp1.setRelease_sec(release_sec, fs_Hz);     comp2.setRelease_sec(release_sec, fs_Hz);
}
