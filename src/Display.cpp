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
    // Serial.printf("%0.2f,%0.2f,-90,90\n", status_->pitch_rad * RAD_TO_DEG, status_->pitch_rad_acc * RAD_TO_DEG);

    TFT_eSprite sprite(&disp_);
    sprite.createSprite(200, 125);
    sprite.fillSprite(TFT_BLACK);
    sprite.setTextColor(TFT_WHITE);
    sprite.setTextFont(2);
    sprite.setTextSize(1);

    sprite.setCursor(15, 5);
    sprite.printf("%0.1f deg", status_->pitch_rad * RAD_TO_DEG);
    sprite.setCursor(110, 5);
    sprite.printf("%0.1f mm/s", status_->speed);

    float battery_pct = (constrain(status_->v_in_volts, 9.0, 12.10) - 9) * 100 / (12.0 - 9.0);
    sprite.setCursor(15, 20);
    sprite.printf("%0.1f V (%0.0f %%)", status_->v_in_volts, battery_pct);

    sprite.setCursor(15, 42);
    sprite.print("Pitch");
    sprite.drawLine(15, 57, 47, 57, TFT_WHITE);
    sprite.setCursor(15, 60);
    sprite.printf("P: %0.1f", status_->pitch_P);
    sprite.setCursor(15, 75);
    sprite.printf("I: %0.1f", status_->pitch_I);
    sprite.setCursor(15, 90);
    sprite.printf("D: %0.1f", status_->pitch_D);

    sprite.setCursor(110, 42);
    sprite.print("Pos");
    sprite.drawLine(110, 57, 155, 57, TFT_WHITE);
    sprite.setCursor(110, 60);
    sprite.printf("P: %0.5f", status_->speed_P);
    sprite.setCursor(110, 75);
    sprite.printf("I: %0.5f", status_->speed_I);
    sprite.setCursor(110, 90);
    sprite.printf("D: %0.5f", status_->speed_D);

    if (status_->loop_freq_warning)
    {
        Serial.println("LOOP FREQUENCY WARNING");
        sprite.setCursor(15, 105);
        sprite.setTextColor(TFT_RED);
        sprite.printf("LOOP FREQ WARNING");
        sprite.setTextColor(TFT_WHITE);
    }
    sprite.pushSprite(3, 3);
}
