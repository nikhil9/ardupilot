#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include "ch.h"

class AP_PPK_FW {
public:

    AP_PPK_FW(void);

    void init();
    void update();

private:

//    AP_Logger logger;

};

extern const AP_HAL::HAL& hal;
