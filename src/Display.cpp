#include "Display.h"

void Display::begin()
{
    disp_.begin();
    disp_.setRotation(rotation_);
    disp_.fillScreen(TFT_BLACK);
    disp_.drawRoundRect(0, 0, screen_w_, screen_h_, 10, TFT_ORANGE);
}

volatile void Display::update()
{
    // Serial.printf("%0.2f,%0.2f,-90,90\n", status_->pitch_rad, status_->pitch_rad_acc);

    const int buff = 3;
    // w and h switched because in landscape mode
    const int sprite_w = TFT_HEIGHT - 2 * buff;
    const int sprite_h = TFT_WIDTH - 2 * buff;

    TFT_eSprite sprite(&disp_);
    sprite.createSprite(sprite_w, sprite_h);
    sprite.fillSprite(TFT_BLACK);
    sprite.setTextColor(TFT_WHITE);
    sprite.setTextFont(2);
    sprite.setTextSize(1);

    // print pitch angle
    sprite.setCursor(15, 5);
    sprite.printf("%0.1f deg", status_->pitch_rad);

    // Print position
    sprite.setCursor(15, 20);
    sprite.printf("%0.1f mm", status_->position);

    // Print battery voltage
    sprite.setCursor(110, 5);
    sprite.printf("%0.1f V", status_->batt_volts);

    // Draw box indicating speed:
    const int max_abs_speed = 300;
    const int box_w = 20;
    const int rbuff = 5; // Gap between box and right edge of sprite
    int box_h = abs(status_->speed) / max_abs_speed * sprite_h / 2;
    int box_y = status_->speed < 0 ? sprite_h / 2 : sprite_h / 2 - box_h;
    sprite.drawRect(sprite_w - box_w - rbuff, 0, box_w, sprite_h, TFT_DARKGREY);
    sprite.fillRect(sprite_w - box_w - rbuff, box_y, box_w, box_h, TFT_DARKGREY);
    sprite.drawLine(sprite_w - box_w - rbuff, sprite_h / 2, sprite_w - rbuff, sprite_h / 2, TFT_WHITE);

    sprite.setCursor(15, 42);
    sprite.print("Pitch:");
    sprite.drawLine(15, 57, 47, 57, TFT_WHITE);
    sprite.setCursor(15, 60);
    sprite.printf("P: %0.1f", status_->pitch_kP);
    sprite.setCursor(15, 75);
    sprite.printf("I: %0.1f", status_->pitch_kI);
    sprite.setCursor(15, 90);
    sprite.printf("D: %0.1f", status_->pitch_kD);

    sprite.setCursor(110, 42);
    sprite.print("Position:");
    sprite.drawLine(110, 57, 155, 57, TFT_WHITE);
    sprite.setCursor(110, 60);
    sprite.printf("P: %0.5f", status_->position_kP);
    sprite.setCursor(110, 75);
    sprite.printf("I: %0.5f", status_->position_kI);
    sprite.setCursor(110, 90);
    sprite.printf("D: %0.5f", status_->position_kD);

    sprite.pushSprite(buff, buff);
}
