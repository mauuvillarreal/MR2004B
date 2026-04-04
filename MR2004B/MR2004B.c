#include <stdio.h>
#include "pico/stdlib.h"

const int ledPin = 25;

const int btnPin = 15;
const int kickPin = 22;
int btnState;
int lastBtnState = 1;

unsigned long lastDebounceBtnTime = 0;
unsigned long debounceBtnDelay = 50;

int main()
{
    stdio_init_all();
    sleep_ms(2000); // allow USB serial to connect

    gpio_init(ledPin);
    gpio_set_dir(ledPin, GPIO_OUT);
    gpio_put(ledPin, 1);

    gpio_init(btnPin);
    gpio_set_dir(btnPin, GPIO_IN);
    gpio_pull_up(btnPin); // IMPORTANT

    gpio_init(kickPin);
    gpio_set_dir(kickPin, GPIO_OUT);



    while (true) {
        int reading = gpio_get(btnPin);

        if (reading != lastBtnState) {
            lastDebounceBtnTime = to_ms_since_boot(get_absolute_time());
        }

        if ((to_ms_since_boot(get_absolute_time()) - lastDebounceBtnTime) > debounceBtnDelay) {

            if (reading != btnState) {
                btnState = reading;

                if (btnState == 0) {
                    printf("Button Pressed\n");
                    gpio_put(kickPin, 1);
                    sleep_ms(100);
                    gpio_put(kickPin, 0);
                }
            }
        }

        lastBtnState = reading;
        sleep_ms(10);
    }
}