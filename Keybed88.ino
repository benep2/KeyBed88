//################################################
//#     Keybed Controller pi pico                #
//#     By Benedito Portela - benep2@gmail.com   #
//#            2025, Jan   Version: 1.00         #
//################################################
//

#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <MIDI.h>
#define ROFF 9
#define RVel press[90]
// USB MIDI object
Adafruit_USBD_MIDI usb_midi;
// Create a new instance of the Arduino MIDI Library,
// and attach usb_midi as the transport.
   MIDI_CREATE_INSTANCE(Adafruit_USBD_MIDI, usb_midi, Midi);

unsigned long long press[90];
uint8_t rd,rdu,Key;
uint8_t pressD[90];
//Pedal
uint8_t pedal=0,pread=0;
unsigned long temp;
#define PedalPin 24
//Pitch Bend
int bend,bendo,pith,pitho;
#define OF 8

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
  Serial.begin(115200);
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
  pinMode(PedalPin,INPUT_PULLUP); //Pedal pin
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
        rd=digitalRead(y+ROFF); //ler contato low
            Key=(x*6)+y; // Contato lido low
            if ((rd==0) && (press[Key]==0)) {
              press[Key]=micros();
              digitalWrite(25,1);
            } 
          if ((rd==1) && (press[Key]>0)) {
            press[Key]=0;
            NTOFF(Key+19);
            digitalWrite(25,0);
             }
          rdu=digitalRead(y+ROFF+6); //ler contato up
            if ((rdu==0) &&(pressD[Key]==0))
              {
               pressD[Key]=1;
               //CHATGPT
                uint32_t dt = micros() - press[Key];
                if (dt < 4500) dt = 4500;
                if (dt > 100000) dt = 100000;
                float x = (float)(dt - 4500) / (100000 - 4500);
                x = sqrt(x);           // compressão de ação de martelo
                float v = 1.0 - x;     // rápido = forte
                v = pow(v, 1.3);       // ajuste fino (opcional)
                uint8_t vel = (uint8_t)(v * 127.0);
                if (vel < 1) vel = 1;
                if (vel > 127) vel = 127;
               NTON(Key+19,vel); 
              }
          if ((rdu==1) && (pressD[Key]>0)) {
            pressD[Key]=0;
                    
          }

      }//fim for y
      
  } //fim for x   
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
//Bend

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
      //Serial.println(bend);

   } 


}//end if pedal
//Pitch Bend


} //End Main loop 
// Envio das mensagens para as duas portas
void NTON( unsigned char d1, unsigned char d2 )
   {
          Midi.sendNoteOn(d1,d2, 1);
    }
//Note Off    
void NTOFF( unsigned char d1)
   {
          Midi.sendNoteOff(d1,0, 1);
    }
//CC
void CC( unsigned char d1, unsigned char d2 )
   {
          Midi.sendControlChange(d1,d2, 1);
    }
void Pitch( int d1 )
   {
          Midi.sendPitchBend(d1,1);
    }