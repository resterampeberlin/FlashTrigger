//-----------------------------------------------------------------------------|
//  - PROOF OF CONCEPT -                                                       |
//                                                                             | 
// Arduino with nRF24L01+ acting as a flash trigger for the                    |
// Elinchrom EL-Skyport system (emulates the EL-Skyport Receiver)              |
// * Starts up nRF24L01+ with the correct parameter set for the                |
//   Elinchrom Skyport System.                                                 |
// * Receives flash trigger message                                            |
//                                                                             |
// http://resterampeberlin.wordpress.com                                       |
// 2016                                                                        |
// Share/use as you like, but please give credit ;)                            |
//-----------------------------------------------------------------------------|

#include <printf.h>
#include <SPI.h>
#include "RF24.h"

#define TRIGGER_PIN   8       // LED connected to Arduino pin 8
#define CE_PIN        9
#define CS_PIN        10
#define INT_PIN       2

RF24 radio(CE_PIN, CS_PIN);

#define CHANNEL       0 // Skyport Channel 1

const uint8_t skyport_channel[] = {
          0x38, 0x3a, 0x3c, 0x3e, 0x45, 0x47, 0x49, 0x4B };     

const uint8_t skyport_address[] = {0x8f, 0xcb, 0xad};

const uint8_t skyport_group[] = {
          0xFF, 0xC0, 0xD0, 0xE0, 0xF0 };

const uint8_t skyport_cmd_trigger   = 0x00;
const uint8_t skyport_cmd_plus      = 0x01;
const uint8_t skyport_cmd_minus     = 0x02;

/*
 * Interrupt handler
*/

void check_radio(void)                                // Receiver role: Does nothing!  All the work is in IRQ
{ 
  bool tx,fail,rx;
  uint8_t data[2];
  static unsigned long delta;

  radio.whatHappened(tx,fail,rx);
  Serial.println("");

  if ( tx ) {                                         // Have we successfully transmitted?
      Serial.println(F("Send:OK"));
  }
  
  if ( fail ) {                                       // Have we failed to transmit?
      Serial.println(F("Something went wrong ..."));
  }
  
  if ( rx || radio.available()) {                      // Did we receive a message?
    while (radio.available()) {
      radio.read(&data, sizeof(data));

      // Time
      Serial.print(millis());
      Serial.print("\t");

      // Deltatime
      Serial.print(millis()-delta);
      delta=millis();
      Serial.print("\t");
      
      // Value
      Serial.print(data[0], HEX);
      Serial.print(data[1], HEX);
      Serial.print("\t");

      // Group
      for (int i=0; i<sizeof(skyport_group)/sizeof(skyport_group[0]); i++) {
        Serial.print(data[0] == skyport_group[i] ? "x" : "");  
        Serial.print("\t");
      }

      // Command
      Serial.print(data[1] == skyport_cmd_trigger ? "Trigger" : "");  
      Serial.print(data[1] == skyport_cmd_plus ? "Plus" : "");  
      Serial.print(data[1] == skyport_cmd_minus ? "Minus" : "");  
     
      // Trigger flash
      if (data[1] == skyport_cmd_trigger) {
        Serial.print("+");
        digitalWrite(TRIGGER_PIN, HIGH);
        
        // wait 100 us (16 x NOP @ 16Mhz)
        for (uint32_t i=0; i<99; i++) {
          __asm__(
             "nop\n\t" "nop\n\t" "nop\n\t" "nop\n\t"
             "nop\n\t" "nop\n\t" "nop\n\t" "nop\n\t"
             "nop\n\t" "nop\n\t" "nop\n\t" "nop\n\t"
             "nop\n\t" "nop\n\t" "nop\n\t" "nop\n\t" );
        }

        digitalWrite(TRIGGER_PIN, LOW);
        Serial.print("-");
      }

      Serial.println("");
    }
  }
}

/*
 * Setup routine
*/

void setup() {
  // log
  Serial.begin(115200);
  Serial.println("starting setup");

  // configure I/O
  pinMode(TRIGGER_PIN, OUTPUT); 
  digitalWrite(TRIGGER_PIN, LOW);

  // for stabilty reasons wait 100ms
  delay(100);  
  
  // configure radio
  radio.begin();
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_1MBPS);  // speed mode
  radio.setChannel(skyport_channel[CHANNEL]);

  // wait for the radio - just in case
  delay (5);

  // check if channel was accepted
  if (radio.getChannel() == skyport_channel[CHANNEL]) {
    radio.setAddressWidth(3);
    radio.setRetries(0,0);
    radio.setPayloadSize(2);
    radio.setCRCLength(RF24_CRC_8);
    radio.setAutoAck(false);
  
    // print configuration
    printf_begin();
    radio.printDetails();
  
    // Attach interrupt handler to interrupt #0 (using pin 2) 
    delay(50);
    attachInterrupt(digitalPinToInterrupt(INT_PIN), check_radio, LOW);             
  
    radio.openReadingPipe(1, skyport_address);
    radio.startListening(); 
  
    Serial.println("setup complete");
  
    // Show display
    Serial.println("Time\tDelta\tVal\tAll\tA\tB\tC\tD\tCmd");
    Serial.println("-----------------------------------------------------------------------");
   }
  else {
    Serial.println("channel not accepted - probably no radio found");  
  }
}

/*
 * Main loop, basically does nothing
*/

void loop() {
  Serial.print("*");
  delay(1000);
}
