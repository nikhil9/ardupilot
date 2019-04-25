#include <AP_HAL/AP_HAL.h>
#include "AP_PPK.h"
#include "hal.h"

//extern const AP_HAL::HAL &hal;
//const AP_HAL::HAL& hal = AP_HAL::get_HAL();


AP_PPK_FW::AP_PPK_FW(void)
//    : logger()
{
}

AP_PPK_FW ppk;

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

void setup(void)
{
    ppk.init();
}

void loop(void)
{
    ppk.update();
}


void AP_PPK_FW::init()
{
    hal.uartA->begin(115200, 32, 128);
    hal.uartB->begin(115200, 32, 128);

    load_parameters();

    serial_manager.init();

    gps.init(serial_manager);
}


void AP_PPK_FW::update()
{

    static uint32_t last_led_ms;
    uint32_t now = AP_HAL::millis();
    if (now - last_led_ms > 1000) {
        last_led_ms = now;
        palToggleLine(HAL_GPIO_PIN_LED_GREEN);
        hal.uartA->println("led");
    }
    hal.scheduler->delay(2);


}

AP_HAL_MAIN();
