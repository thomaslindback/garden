#ifndef __LED_DISP_FKN__
#define __LED_DISP_FKN__

#include "Adafruit_LEDBackpack.h"
#include <Adafruit_GFX.h>
#include <Arduino.h>
#include <Wire.h>

static const uint8_t PROGMEM
    smile_bmp[] = {B00111100, B01000010, B10100101, B10000001,
                   B10100101, B10011001, B01000010, B00111100},
    circle_1[] = {B00111100, B01000010, B10000001, B10000001,
                  B10000001, B10000001, B01000010, B00111100},
    circle_2[] = {B00111100, B01111110, B11100111, B11000011,
                  B11000011, B11100111, B01111110, B00111100},
    shower_head[] = {B00011000, B00011000, B00011000, B00000000,
                     B00101000, B01010100, B10101010, B01010101},
    wait[] = {B11111111,
              B01111110,
              B00111100,
              B00011000,
              B00011000,
              B00111100,
              B01111110,
              B11111111};

class LedMatrix {
public:
  LedMatrix() { matrix = new Adafruit_8x8matrix(); }
  ~LedMatrix() { delete matrix; }
  void connect(int addr) { matrix->begin(addr); }
  void draw_one_char_to_disp(char *txt);
  void draw_circle();
  void draw_shower();
  void draw_wait();
  void draw_heads_up_seq(uint16_t delay);
  void clear();

private:
  Adafruit_8x8matrix *matrix;
};

#endif