
#include "led_matrix.h"

void LedMatrix::draw_one_char_to_disp(const char *txt) {
  matrix->setTextSize(1);
  matrix->setTextWrap(false); // we dont want text to wrap so it scrolls nicely
  matrix->setTextColor(LED_ON);
  matrix->setCursor(1, 0);
  matrix->clear();
  matrix->print(txt);
  matrix->writeDisplay();
}

void LedMatrix::draw_circle() {
  matrix->drawBitmap(0, 0, circle_2, 8, 8, LED_ON);
  matrix->writeDisplay();
}

void LedMatrix::draw_wait() {
  matrix->drawBitmap(0, 0, wait, 8, 8, LED_ON);
  matrix->writeDisplay();
}

void LedMatrix::draw_shower() {
  matrix->drawBitmap(0, 0, shower_head, 8, 8, LED_ON);
  matrix->writeDisplay();
}

void LedMatrix::draw_heads_up_seq(uint16_t delay) {
  for (int i = 0; i < 4; i++) {
    matrix->clear();
    matrix->drawRect(i, i, 8 - 2 * i, 8 - 2 * i, LED_ON);
    matrix->writeDisplay();
    ::delay(delay);
  }
  matrix->clear();
  matrix->writeDisplay();
}

void LedMatrix::clear() { 
  matrix->clear(); 
  matrix->writeDisplay();
}
