#pragma once

namespace external_led {

void init(int red_pin,
          int green_pin,
          bool red_active_high = true,
          bool green_active_high = true);

void setRed(bool on);
void setGreen(bool on);
void setGreenBreathing(bool enabled);
void update();

} // namespace external_led
