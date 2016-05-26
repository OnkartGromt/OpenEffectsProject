/*
FunkyFellow1_1.ino
Filter effect using state variable filters. 
Thanks to Paul Stoffregen, www.PJRC.com, that made the great audio library fro Teensy! Plus all the other great stuff!
This code can be loaded into a Teensy3.1/3.2 with a Audio sheild. Use a I2C 64x128 OLED screen to show the graphics. 
All in all use the OpenEffectsBox from OnkartGromt. 
 */
#include <EEPROM.h> 
#include <Bounce.h> 
#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

// GUItool: begin automatically generated code
AudioSynthWaveform       waveform1;      //xy=106,573
AudioSynthWaveformDc     dc1;            //xy=109,473
AudioInputI2S            i2s1;           //xy=140,314
AudioSynthWaveformDc     dc2;            //xy=140.88888549804688,358.8888854980469
AudioEffectMultiply      multiply2;      //xy=248,532
AudioAnalyzePeak         peak3;          //xy=254,573
AudioEffectMultiply      multiply3;      //xy=298.8888854980469,313.8888854980469
AudioMixer4              mixer3;         //xy=393,492
AudioFilterStateVariable filter1;        //xy=456,147
AudioEffectMultiply      multiply1;      //xy=495,420
AudioAnalyzePeak         peak1;          //xy=673,420
AudioMixer4              mixer2;         //xy=759.0000915527344,88
AudioFilterStateVariable filter2;        //xy=759,151
AudioFilterStateVariable filter3;        //xy=759,204
AudioFilterStateVariable filter4;        //xy=759,257
AudioMixer4              mixer1;         //xy=891.0000305175781,210
AudioMixer4              mixer4;         //xy=1187,295
AudioAnalyzeFFT1024      fft1024_1;      //xy=1385,389
AudioAnalyzePeak         peak2;          //xy=1394,297
AudioOutputI2S           i2s2;           //xy=1394,342
AudioConnection          patchCord1(waveform1, 0, multiply2, 1);
AudioConnection          patchCord2(waveform1, peak3);
AudioConnection          patchCord3(dc1, 0, multiply2, 0);
AudioConnection          patchCord4(dc1, 0, mixer3, 0);
AudioConnection          patchCord5(i2s1, 0, multiply3, 0);
AudioConnection          patchCord6(dc2, 0, multiply3, 1);
AudioConnection          patchCord7(multiply2, 0, mixer3, 1);
AudioConnection          patchCord8(multiply3, 0, filter1, 0);
AudioConnection          patchCord9(multiply3, fft1024_1);
AudioConnection          patchCord10(multiply3, 0, mixer4, 3);
AudioConnection          patchCord13(multiply3, 0, multiply1, 0);
AudioConnection          patchCord14(mixer3, 0, multiply1, 1);
AudioConnection          patchCord15(filter1, 0, filter2, 0);
AudioConnection          patchCord16(filter1, 0, mixer2, 0);
AudioConnection          patchCord17(filter1, 1, filter3, 0);
AudioConnection          patchCord18(filter1, 1, mixer2, 1);
AudioConnection          patchCord19(filter1, 2, filter4, 0);
AudioConnection          patchCord20(filter1, 2, mixer2, 2);
AudioConnection          patchCord21(multiply1, peak1);
AudioConnection          patchCord22(multiply1, 0, filter1, 1);
AudioConnection          patchCord23(multiply1, 0, filter2, 1);
AudioConnection          patchCord24(multiply1, 0, filter3, 1);
AudioConnection          patchCord25(multiply1, 0, filter4, 1);
AudioConnection          patchCord26(mixer2, 0, mixer4, 0);
AudioConnection          patchCord27(filter2, 0, mixer1, 0);
AudioConnection          patchCord28(filter3, 1, mixer1, 1);
AudioConnection          patchCord29(filter4, 2, mixer1, 2);
AudioConnection          patchCord30(mixer1, 0, mixer4, 1);
AudioConnection          patchCord31(mixer4, peak2);
AudioConnection          patchCord32(mixer4, 0, i2s2, 0);
AudioConnection          patchCord33(mixer4, 0, i2s2, 1);
AudioControlSGTL5000     sgtl5000_1;     //xy=662.0000305175781,536.0000305175781
// GUItool: end automatically generated code


//Pinout board rev1
int Pot1 = A6;    // select the input pin for the potentiometer
int Pot2 = A3;    // select the input pin for the potentiometer
int Pot3 = A2;    // select the input pin for the potentiometer
int Pot4 = A1;    // select the input pin for the potentiometer
int CV1 = A10;
int CV2 = A11;

int Tap1 = 0;
int Tap2 = 1;

int SW1 = A12;
int SW2 = A13;

int ledPin1 = 3;      // select the pin for the LEDonoff
int ledPin2 = 4;      // select the pin for the LEDyellow
int ledPin3 = 5;      // select the pin for the LEDmode/tap

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
int Tap1Timer = 0;
int Tap2Timer = 0;
float margin  = 0.1;
int modulationMode = 0;
boolean Tap2Pressed = 0;
int filterOrder = 24;
int GraphFreq = 0;
float ffund = 0;

// Variables 
int ledStateONOFF = LOW;         // the current state of the output pin
int buttonState0;             // the current reading from the input pin
int lastButtonState0 = LOW;   // the previous reading from the input pin
int ledStateBOOST = LOW;         // the current state of the output pin
int buttonState1;             // the current reading from the input pin
int lastButtonState1 = LOW;   // the previous reading from the input pin

unsigned long lastDebounceTime0 = 0;  // the last time the output pin was toggled
unsigned long lastDebounceTime1 = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

float peak = 0.0;
float peak_2 = 0.0;
float peak_3 =0.0;
int spanfreq = 500;
int spanfreqOld = 500;
int offsetfreq = 201;
int movingfreq = 500;
int octave = 7;
int octaveOld = 0;
float level = 0.85;
float modFreqTap = 2.0;
float modFreq = 3;
float modFreqOld = 2;
float InLevel = 0;

const int myInput = AUDIO_INPUT_LINEIN;

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
Bounce OnOff = Bounce(Tap1, 30 ); 
Bounce BoostOnOff = Bounce(Tap2, 30); 
elapsedMillis tapInterval;  //timer that is used for tap tempo

void setup() {
  AudioMemory(100);
  InLevel = float(EEPROM.read(0)/255.0);
  dc2.amplitude(InLevel, 20);
  sgtl5000_1.enable();  // Enable the audio shield
  sgtl5000_1.inputSelect(myInput);
  sgtl5000_1.volume(0.7);
  sgtl5000_1.lineInLevel(0);   // 3.12Vp-p
  sgtl5000_1.lineOutLevel(13); // 3.16Vp-p
  fft1024_1.windowFunction(AudioWindowHanning1024);
  
  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  pinMode(ledPin3, OUTPUT);
  pinMode(Tap1, INPUT);
  pinMode(Tap2, INPUT);
  mixer3.gain(0, 1);
  mixer3.gain(1, 0);
  mixer4.gain(0, 0);
  mixer4.gain(1, 0);
  mixer4.gain(3, 2);  
  mixer1.gain(0, 1);    //24order filter
  mixer1.gain(1, 1);
  mixer1.gain(2, 1);
  mixer1.gain(3, 0);
  mixer2.gain(0, 1);    //12 order filter
  mixer2.gain(1, 1);
  mixer2.gain(2, 1);
  mixer2.gain(3, 0);
  waveform1.begin(2,2,WAVEFORM_SINE);
  sgtl5000_1.dacVolume (level);
  sgtl5000_1.autoVolumeControl(0,0,0,0.7,50,100);  //maxGain,response,hard limit,threshold,attack, decay
  sgtl5000_1.autoVolumeDisable();
  Serial.begin(115200);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x64)
  display.clearDisplay();
  delay(100);
}

//Main loop
void loop() {
  OnOff.update();      //update the bounce
  BoostOnOff.update();

  // read the value from the sensor:
  Pot1Value = analogRead(Pot1);    //Sense
  Pot2Value = analogRead(Pot2);    //Resonance
  Pot3Value = analogRead(Pot3);    //Freq
  Pot4Value = analogRead(Pot4);    //Mode
  Tap1Value = digitalRead(Tap1);   //On off
  Tap2Value = digitalRead(Tap2);   //Mode/Tap
  SW1Value = analogRead(SW1);      //Filter char
  SW2Value = analogRead(SW2);      //Direction
  CV1Value = analogRead(CV1);      //CV input
  CV2Value = analogRead(CV2);      //EXP input
  
  Pot1Value = mapfloat(Pot1Value, 0, 1023, 1, 0);        //Sense
  Pot2Value = mapfloat(Pot2Value, 0, 1023, 4, 0.7);      //Resonance
  GraphFreq = map(Pot3Value, 0, 1023, 90, 30);           //Moving point of Graph
  Pot3Value = map(Pot3Value, 0, 1023, 1500, 150);        //Freq
  modFreq  = mapfloat(Pot4Value, 0, 1023, 10.0, 1.0);    //ModFreq
  SW1Value = map(SW1Value, 0, 1023, 5, 0);               //Filter char
  SW1Value = constrain(SW1Value, 1, 5);
  SW2Value = map(SW2Value, 0, 1023, 5, 0);               //Direction
  SW2Value = constrain(SW2Value, 1, 5);
  CV1Value = mapfloat(CV1Value, 0, 1023, 10, 1);       //Sine modulation CV pedal input
  CV2Value = map(CV2Value, 0, 600, 1500, 200);           //Freq expression pedal  

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
  digitalWrite(ledPin1, ledStateONOFF); 
  lastButtonState0 = buttonState0;  //save readings to next round
  lastButtonState1 = buttonState1; 

//********************SENSE handling**********************
  //Sets the sense multiplication factor
  if (SW2Value == 3)dc1.amplitude(Pot1Value);
  else if (SW2Value == 5)dc1.amplitude(-Pot1Value);
  
  //read the signal input peak
  if (peak1.available())  peak = peak1.read();
  if (peak2.available())  peak_2 = peak2.read();    //Output peak
  if (peak3.available())  peak_3 = peak3.read();
  
  if (modulationMode == 0){
    analogWrite(ledPin2, peak_2*400);    //yellow LED
    if (Pot4Value <= 512)filterOrder = 12;
    else filterOrder = 24;
    }
  else analogWrite(ledPin2, (peak_3)*100);  //modulation active
  
//******************** SET filter handling**********************  
  if (CV1Value <8.0 && modulationMode == 1){  //Check if CVinput is active
    modFreq = CV1Value;    //expression pedal overrides the potmeter
    }
  
  if ((CV2Value <80)||(CV2Value > 1400)) spanfreq = Pot3Value;  //Check if exp pedal is active
  else spanfreq = CV2Value;
  
  if (SW2Value == 3) movingfreq = peak*2 * spanfreq + spanfreq ;
  else if (SW2Value == 5) movingfreq = -peak *2* spanfreq + spanfreq ;
  else if (SW2Value == 1) movingfreq = spanfreq;    //not moving filter

if ((spanfreq > spanfreqOld+ 4) || (spanfreq < spanfreqOld - 4)){
  filter1.frequency(spanfreq);
  filter2.frequency(spanfreq);
  filter3.frequency(spanfreq);
  filter4.frequency(spanfreq);
}
if ((Pot2Value > Pot2ValueOld+ 0.1) || (Pot2Value < Pot2ValueOld - 0.1)){
  filter1.resonance(Pot2Value);
  filter2.resonance(Pot2Value);
  filter3.resonance(Pot2Value);
  filter4.resonance(Pot2Value);  
}
if ((octave > octaveOld+ 1) || (octave < octaveOld - 1)){
  filter1.octaveControl(octave);
  filter2.octaveControl(octave);
  filter3.octaveControl(octave);
  filter4.octaveControl(octave);
}



//********************Activate modulation handling**********************
if (!Tap2Value && (Tap2Value == Tap2ValueOld) && modulationMode == 0 && Tap2Pressed == 0){ //turn on modulation
  //Serial.print("TIME0\n");
  Tap2Timer++;
  if (Tap2Timer > 15){
    mixer3.gain(0,0);
    mixer3.gain(1,1);
    modulationMode = 1;  
    analogWrite(ledPin2,255);
    digitalWrite(ledPin3,HIGH);
    Tap2Pressed = 1;
  }
}
else if (!Tap2Value && (Tap2Value == Tap2ValueOld) && modulationMode == 1 && Tap2Pressed == 0){ //turn off modulation
  //Serial.print("Modulation\n");
  Tap2Timer++;
  if (Tap2Timer > 15){
    mixer3.gain(0,1);
    mixer3.gain(1,0);
    modulationMode = 0;
    analogWrite(ledPin2,255);
    digitalWrite(ledPin3,LOW);
    Tap2Pressed = 1;
  }
}
else if (!Tap2Value && modulationMode == 1 && Tap2Timer <15){
  if (tapInterval <2000 && tapInterval > 100);
    modFreqTap = 1/(tapInterval*0.001);
    tapInterval = 0;
    Serial.println(modFreqTap);
    waveform1.frequency(modFreqTap);
  }
else if (Tap2Value){
  Tap2Timer = 0;
  Tap2Pressed = 0;
  }
//********* Update modulation freq ********************
if ((modFreq > modFreqOld+ 0.1) || (modFreq < modFreqOld - 0.1)){
  waveform1.frequency(modFreq);
  }
  
//BYPASS MODE 
  if (ledStateONOFF == 0){
      mixer4.gain(0, 0);
      mixer4.gain(1, 0);
      mixer4.gain(2, 0);
      mixer4.gain(3, 2);
  }      
  else {  
      mixer4.gain(3, 0);  //Always turn off bypass signal
    if (SW1Value == 3) {  //HP
      if (filterOrder == 12) { //   12db/dec
        mixer4.gain(0, 2);
        mixer4.gain(1, 0);
        mixer2.gain(0, 0);
        mixer2.gain(1, 0);  
        mixer2.gain(2, 1);         
        }
      else if (filterOrder == 24) {    //24db/dec
        mixer4.gain(0, 0);
        mixer4.gain(1, 2);
        mixer1.gain(0, 0);
        mixer1.gain(1, 0);  
        mixer1.gain(2, 1);         
      }   
    }
    else if (SW1Value == 1) {  //BP  
       if (filterOrder == 12) { //   12db/dec
        mixer4.gain(0, 2);
        mixer4.gain(1, 0);
        mixer2.gain(0, 0);
        mixer2.gain(1, 1);  
        mixer2.gain(2, 0);         
        }
      else if (filterOrder == 24) {    //24db/dec
        mixer4.gain(0, 0);
        mixer4.gain(1, 2);
        mixer1.gain(0, 0);
        mixer1.gain(1, 1);  
        mixer1.gain(2, 0);         
      }   
    }
    else if (SW1Value == 5) { //LowPass
       if (filterOrder == 12) { //   12db/dec
        mixer4.gain(0, 2);
        mixer4.gain(1, 0);
        mixer2.gain(0, 1);
        mixer2.gain(1, 0);  
        mixer2.gain(2, 0);         
        }
      else if (filterOrder == 24) {    //24db/dec
        mixer4.gain(0, 0);
        mixer4.gain(1, 2);
        mixer1.gain(0, 1);
        mixer1.gain(1, 0);  
        mixer1.gain(2, 0);         
      }   
    }
  }  
display.clearDisplay();

if (!Tap1Value && (Tap1Value == Tap1ValueOld)){    //When Mode/tap is held down you can adjust the input gain of the input
 // Serial.print("TIME");
  Tap1Timer++;
  if (Tap1Timer > 15){
    InLevel = analogRead(Pot1);
    InLevel = mapfloat(InLevel,0,1023,1,0.1);
    dc2.amplitude(InLevel,10);
    delay(100);
    analogWrite(ledPin2, 255);  
    //display.clearDisplay();   
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
  analogWrite(ledPin2, peak*100);
}
if(fft1024_1.available()){        //Display FFT on screen
 // display.clearDisplay();
  for (int i=0; i<42; i++) {
    float fft = fft1024_1.read(i)*512;
    //Serial.print(fft);
    //Serial.print(",");
    display.drawLine(i*3, 64, i*3, 64-fft, WHITE);
    display.drawLine(i*3+1, 64, i*3+1, 64-fft, WHITE);
    display.drawLine(i*3+2, 64, i*3+2, 64-fft, WHITE);
  }
}  
//  display.clearDisplay();
  GraphFreq = movingfreq/15;
 if(SW1Value ==5){    //LP
    display.drawLine(0, 32, GraphFreq-5, 32, WHITE);
    display.drawLine(GraphFreq-5, 32, GraphFreq, 35-Pot2Value*4, WHITE);
    display.drawLine(GraphFreq, 35-Pot2Value*4, GraphFreq+5, 32, WHITE);
    display.drawLine(GraphFreq+5, 32, GraphFreq+(48-filterOrder), 64, WHITE);
 }   
 else if(SW1Value == 3){    //HP
    display.drawLine(128, 32, GraphFreq+5, 32, WHITE);
    display.drawLine(GraphFreq+5, 32, GraphFreq, 35-Pot2Value*4, WHITE);
    display.drawLine(GraphFreq, 35-Pot2Value*4, GraphFreq-5, 32, WHITE);
    display.drawLine(GraphFreq-5, 32, GraphFreq-(48-filterOrder), 64, WHITE);
 }   
  else {    //BP
    display.drawLine(GraphFreq-(48-filterOrder), 64, GraphFreq-5, 32, WHITE);
    display.drawLine(GraphFreq+5, 32, GraphFreq, 35-Pot2Value*4, WHITE);
    display.drawLine(GraphFreq, 35-Pot2Value*4, GraphFreq-5, 32, WHITE);
    display.drawLine(GraphFreq+5, 32, GraphFreq+(48-filterOrder), 64, WHITE);
 }  
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(80, 10);
  display.print("fc:");
  display.print(movingfreq); 
  display.display();
/*
   Serial.print(" CV1Value: ");
   Serial.print(CV1Value);
   Serial.print(" octave: ");
   Serial.print(octave);   
   Serial.print(" CV2: ");
   Serial.println(CV2Value);   
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
   modFreqOld = modFreq;
   spanfreqOld = spanfreq;
}
