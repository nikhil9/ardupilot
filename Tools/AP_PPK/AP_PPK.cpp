#include <AP_HAL/AP_HAL.h>
#include "AP_PPK.h"
#include "hal.h"

//extern const AP_HAL::HAL &hal;
//const AP_HAL::HAL& hal = AP_HAL::get_HAL();



AP_PPK_FW::AP_PPK_FW(void)
//    : logger()
{
}

static AP_PPK_FW ppk;

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

}


void AP_PPK_FW::update()
{

}

AP_HAL_MAIN();
