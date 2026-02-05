//################################################
//#     Keybed Controller pi pico                #
//#     By Benedito Portela - benep2@gmail.com   #
//#            2026, Fev   Version: 1.02         #
//################################################
//

#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <MIDI.h>
// USB MIDI object
Adafruit_USBD_MIDI usb_midi;
// Create a new instance of the Arduino MIDI Library,
// and attach usb_midi as the transport.
MIDI_CREATE_INSTANCE(Adafruit_USBD_MIDI, usb_midi, Midi);
MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, MidiS);
//Keys
#define ROFF 9 //Read Pin Offset
unsigned long long press[90];
uint8_t rd,rdu,Key;
uint8_t pressD[90];
//Pedal
uint8_t pedal=0,pread=0;
unsigned long temp;
#define PedalPin 21
//Pitch Bend
#define OF 8
int bend,bendo,pith,pitho;
//Pots CC 
uint8_t potCC[3]={07,91,93};// Volume, Reverb, Chorus 
uint16_t pot[3],poto[3]={0,0,0},potr;

void setup()  
{
  
  analogReadResolution(12);//Analog Resolution
    for (uint8_t i=0; i<90; i++) {
        press[i]=0;
        pressD[i]=0;
        }
   //Name of device     
  TinyUSBDevice.setManufacturerDescriptor("Benedito");
  TinyUSBDevice.setProductDescriptor("KeyBed_88");
  TinyUSBDevice.setSerialDescriptor("Keybed-001");
  Midi.begin(MIDI_CHANNEL_OMNI);
  MidiS.begin(MIDI_CHANNEL_OMNI);
  Serial1.begin(31250);
  // Pins multiplex 4 bits 
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(8,OUTPUT);
  pinMode(25,OUTPUT);
// Read pins
  pinMode(9,INPUT_PULLUP);
  pinMode(10,INPUT_PULLUP);
  pinMode(11,INPUT_PULLUP);
  pinMode(12,INPUT_PULLUP);
  pinMode(13,INPUT_PULLUP);
  pinMode(14,INPUT_PULLUP);
  pinMode(15,INPUT_PULLUP);
  pinMode(16,INPUT_PULLUP);
  pinMode(17,INPUT_PULLUP);
  pinMode(18,INPUT_PULLUP);
  pinMode(19,INPUT_PULLUP);
  pinMode(20,INPUT_PULLUP);
  //Pedal Pin
  pinMode(PedalPin,INPUT_PULLUP); 
}

void loop() // Main loop
{
 // Read Keybed
  for  (uint8_t x=0;x<15;x++) {
      digitalWrite(8,(x & 1));// bit 0
      digitalWrite(7,(x & 2));// bit 1
      digitalWrite(6,(x & 4));// bit 2
      digitalWrite(5,(x & 8));// bit 3
      delayMicroseconds(10);
      for ( uint8_t y=0;y<6;y++) {
        rd=digitalRead(y+ROFF); //Read low contact
            Key=(x*6)+y; // Key definiton
            if ((rd==0) && (press[Key]==0)) {
              press[Key]=micros(); //Read time
              digitalWrite(25,1); // turned ON Led 
            } 
          if ((rd==1) && (press[Key]>0)) {
            press[Key]=0;
            NTOFF(Key+19);
            digitalWrite(25,0);
             }
          rdu=digitalRead(y+ROFF+6); //Read up contact
            if ((rdu==0) &&(pressD[Key]==0))
              {
               pressD[Key]=1;
                uint32_t dt = micros() - press[Key];
                if (dt < 4500) dt = 4500;
                if (dt > 100000) dt = 100000;
                float k = (float)(dt - 4500) / (100000 - 4500);
                k = sqrt(k);           // Hammer aciton compress
                float v = 1.0 - k;     // fast = strong
                v = pow(v, 1.3);       // fine adjust (opcional)
                uint8_t vel = (uint8_t)(v * 127.0);
                if (vel < 1) vel = 1;
                if (vel > 127) vel = 127;
               NTON(Key+19,vel); 
              }
          if ((rdu==1) && (pressD[Key]>0)) {
            pressD[Key]=0;
                    
          }

      }//end for y
      
  } //end for x   
//Pedal
if (millis()>=temp+5) {
temp=millis();
pread=digitalRead(PedalPin);
     if ((pread==0) && (pedal==1)){ 
     pedal=0;
     CC(64,0);
     }
if ((pread==1) && (pedal==0)){ 
     pedal=1;
     CC(64,127);
     }
//Pitch Bend
bend=analogRead(26);
   if ((bend>bendo+OF) || (bend<bendo-OF)) {
      bendo=bend;
      if (bend>2390){
          pith=map(bendo,2400,3930,0,8191);
          if (pith>8191) pith=8191;
      }
      if (bend<1880){
          pith=(map(bendo,230,1870,0,8192))-8192;
          if (pith<-8192) pith=-8192;
      }
      if ((bend>1890)&&(bend<2250)) pith=0;
      
      if (pith!=pitho){
        pitho=pith;
        Pitch(pith);
      }
    } //end of bend
//Potentiometers CC
  for (uint8_t c=0;c<3;c++){
     potr=analogRead(27+c);
      if ((potr>poto[c]+28) || (potr<poto[c]-28)) {
         poto[c]=potr;
         potr=map(potr,20,4076,0,128);
          if (potr<0) potr=0;
          if (potr>127) potr=127;
            if (potr!=pot[c]) {
              pot[c]=potr;
              CC(potCC[c],potr);
              }
      }
  } //end of for CC

}//end if pedal

} //End Main loop 
// Send MIDI messengers in USB and MIDI Serial Port
void NTON( unsigned char d1, unsigned char d2 )
   {
          Midi.sendNoteOn(d1,d2, 1);
          MidiS.sendNoteOn(d1,d2, 1);
    }
//Note Off    
void NTOFF( unsigned char d1)
   {
          Midi.sendNoteOff(d1,0, 1);
          MidiS.sendNoteOff(d1,0, 1);
    }
//CC
void CC( unsigned char d1, unsigned char d2 )
   {
          Midi.sendControlChange(d1,d2, 1);
          MidiS.sendControlChange(d1,d2, 1);
    }
    // Pitch Bend
void Pitch( int d1 )
   {
          Midi.sendPitchBend(d1,1);
          MidiS.sendPitchBend(d1,1);
    }