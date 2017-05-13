
//---------------------------------------------------//
//                                                   //
//             Arduino Semaphore Controller          //
//             Deigned for ROSpiBot project          //
//                                                   //
//  Sketch by Andrea Fioroni (andrifiore@gmail.com)  //
//                                                   //
//                  GitHub repo:                     //
// https://github.com/isarlab-department-engineering //
//                                                   //
//---------------------------------------------------//

#include <Adafruit_NeoPixel.h>
#define NEO_PIN 3 // LED pin
#define RED_MS 12000 // red LED duration in ms
#define GREEN_MS 6000 // green LED duration in ms

//----- RED_POS = GREEN_POS = 0 means we're using a single RGB LED, alternating its color
//----- if using 2 different LEDs just set LED COUNT = 2, RED_POS = 0 and GREEN_POS = 1
#define LED_COUNT 1
#define RED_POS 0
#define GREEN_POS 0
Adafruit_NeoPixel leds = Adafruit_NeoPixel(LED_COUNT, NEO_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  
  Serial.begin(9600); // start Serial Port
  
  leds.begin(); // start NeoPixel
  leds.setPixelColor(RED_POS, 0x00, 0x00, 0x00);
  leds.setPixelColor(GREEN_POS, 0x00, 0x00, 0x00);
  
  leds.show();
  
}

void loop() {
  
  if (LED_COUNT == 1) { // single LED => just swap
    
    // Red LED on
    leds.setPixelColor(RED_POS, 0xFF, 0x00, 0x00);
    Serial.println("Red LED on");
    leds.show();
    delay(RED_MS);
    
    // Green LED on
    leds.setPixelColor(GREEN_POS, 0x00, 0xFF, 0x00);
    Serial.println("Green LED on");
    leds.show();
    delay(GREEN_MS);
    
  } else { // 2 LEDs => alternate
    
    // Red LED on
    leds.setPixelColor(RED_POS, 0xFF, 0x00, 0x00);
    Serial.println("Red LED on");
    leds.show();
    delay(RED_MS);
    leds.setPixelColor(RED_POS, 0x00, 0x00, 0x00);
    
    // Green LED on
    leds.setPixelColor(GREEN_POS, 0x00, 0xFF, 0x00);
    Serial.println("Green LED on");
    leds.show();
    delay(GREEN_MS);
    leds.setPixelColor(GREEN_POS, 0x00, 0x00, 0x00);
    
  }
  
}
