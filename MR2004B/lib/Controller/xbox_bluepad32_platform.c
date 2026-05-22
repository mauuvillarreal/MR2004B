#include "xbox_bluepad32_platform.h"
#include "xbox_controller_c_api.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "pico/cyw43_arch.h"
#include "pico/time.h"

#include <uni.h>

#ifndef ARG_UNUSED
#define ARG_UNUSED(x) (void)(x)
#endif

static void platform_init(int argc, const char** argv) {
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    logi("xbox_platform: init\n");
}

static void platform_on_init_complete(void) {
    logi("xbox_platform: init complete\n");

    // Start Bluetooth scanning and autoconnect.
    uni_bt_start_scanning_and_autoconnect_unsafe();

    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
}

static uni_error_t platform_on_device_discovered(
    bd_addr_t addr,
    const char* name,
    uint16_t cod,
    uint8_t rssi
) {
    ARG_UNUSED(addr);
    ARG_UNUSED(cod);
    ARG_UNUSED(rssi);

    if (name) {
        logi("Discovered BT device: %s\n", name);
    }

    return UNI_ERROR_SUCCESS;
}

static void platform_on_device_connected(uni_hid_device_t* d) {
    logi("xbox_platform: device connected: %p\n", d);
    xbox_cpp_on_connected();
}

static void platform_on_device_disconnected(uni_hid_device_t* d) {
    logi("xbox_platform: device disconnected: %p\n", d);
    xbox_cpp_on_disconnected();
}

static uni_error_t platform_on_device_ready(uni_hid_device_t* d) {
    logi("xbox_platform: device ready: %p\n", d);
    return UNI_ERROR_SUCCESS;
}

static uint32_t map_bluepad_buttons(uint32_t bp_buttons, uint32_t bp_misc) {
    uint32_t out = 0;

    if (bp_buttons & BUTTON_A) out |= 1u << 0;
    if (bp_buttons & BUTTON_B) out |= 1u << 1;
    if (bp_buttons & BUTTON_X) out |= 1u << 2;
    if (bp_buttons & BUTTON_Y) out |= 1u << 3;

    if (bp_buttons & BUTTON_SHOULDER_L) out |= 1u << 4;
    if (bp_buttons & BUTTON_SHOULDER_R) out |= 1u << 5;

    if (bp_misc & MISC_BUTTON_BACK) out |= 1u << 6;
    if (bp_misc & MISC_BUTTON_START) out |= 1u << 7;

    if (bp_buttons & BUTTON_THUMB_L) out |= 1u << 8;
    if (bp_buttons & BUTTON_THUMB_R) out |= 1u << 9;

    return out;
}

static void platform_on_controller_data(uni_hid_device_t* d, uni_controller_t* ctl) {
    ARG_UNUSED(d);

    if (ctl->klass != UNI_CONTROLLER_CLASS_GAMEPAD) {
        return;
    }

    uni_gamepad_t* gp = &ctl->gamepad;
    uint32_t mapped_buttons = map_bluepad_buttons(gp->buttons, gp->misc_buttons);

    xbox_cpp_on_gamepad_data(
        gp->axis_x,
        gp->axis_y,
        gp->axis_rx,
        gp->axis_ry,
        gp->brake,
        gp->throttle,
        mapped_buttons,
        gp->misc_buttons,
        gp->dpad
    );
}

static const uni_property_t* platform_get_property(uni_property_idx_t idx) {
    ARG_UNUSED(idx);
    return NULL;
}

static void platform_on_oob_event(uni_platform_oob_event_t event, void* data) {
    ARG_UNUSED(data);
    logi("xbox_platform: OOB event: 0x%04x\n", event);
}

struct uni_platform* xbox_get_bluepad32_platform(void) {
    static struct uni_platform platform = {
        .name = "RoboPortero Xbox Platform",
        .init = platform_init,
        .on_init_complete = platform_on_init_complete,
        .on_device_discovered = platform_on_device_discovered,
        .on_device_connected = platform_on_device_connected,
        .on_device_disconnected = platform_on_device_disconnected,
        .on_device_ready = platform_on_device_ready,
        .on_oob_event = platform_on_oob_event,
        .on_controller_data = platform_on_controller_data,
        .get_property = platform_get_property,
    };

    return &platform;
}
