//################################################
//#     Keybed Controller pi pico                #
//#     By Benedito Portela - benep2@gmail.com   #
//#            2025, Jan   Version: 1.00         #
//################################################
//

#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <MIDI.h>

// USB MIDI object
Adafruit_USBD_MIDI usb_midi;

// Create a new instance of the Arduino MIDI Library,
// and attach usb_midi as the transport.
 //   MIDI_CREATE_INSTANCE(Adafruit_USBD_MIDI, usb_midi, Midi);

uint8_t dado,dado_out; 
uint8_t dadoT,dado1,dado2,dadoC;
unsigned long long press[90];
void setup()  
{
    for (int i=0; i<90; i++) {
        press[i]=0;
        }
 // Midi.begin(MIDI_CHANNEL_OMNI);
  Serial.begin(115200);
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(8,OUTPUT);
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

}

void loop() // run over and over
{
  for  (int x=0;x<15;x++) {
     // x=5;
      digitalWrite(8,(x & 1));// bit 0
      digitalWrite(7,(x & 2));// bit 1
      digitalWrite(6,(x & 4));// bit 2
      digitalWrite(5,(x & 8));// bit 3
      delay (50);

  } //fim for x   
  
}  
// Envio das mensagens para as duas portas
//void SendMidi ( unsigned char d1, unsigned char d2, unsigned char d3, unsigned char d4 )
//   {
//      Midi.send (midi::MidiType(d1) ,d2,d3,d4);  
//    }
      

