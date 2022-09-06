#include <Adafruit_NeoPixel.h>
#define LEDPIN RGB_LED // connect the Data from the strip to this pin on the Arduino
#define NUMBER_PIEXELS 1 // the number of pixels in your LED strip
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUMBER_PIEXELS, LEDPIN, NEO_GRB + NEO_KHZ800);

int wait = 10; // how long we wait on each color (milliseconds)
uint32_t no;

void Init_NeoPixcel() {
  strip.begin();
  strip.setBrightness(255);
}

void NeoPixcel_task() {
  no = 0xfffe;
  strip.setPixelColor(0, Wheel(no));
  strip.show();
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  #ifdef NeoPixcel_ENABLE
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else if(WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  } else {
    WheelPos -= 170;
    return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  }
  #endif
  //return strip.Color(30, 30, 30);
}
