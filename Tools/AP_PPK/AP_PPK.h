#include <AP_HAL/AP_HAL.h>
#include "ch.h"

#include <AP_Param/AP_Param.h>
#include <AP_GPS/AP_GPS.h>
#include "Parameters.h"

class AP_PPK_FW {
public:

    AP_PPK_FW(void);

    void init();
    void update();

private:

    void load_parameters();

    Parameters g;

    AP_SerialManager serial_manager;
    AP_GPS gps;

    // setup the var_info table
    AP_Param param_loader{var_info};

    static const AP_Param::Info var_info[];

    uint32_t last_mag_update_ms;
    uint32_t last_gps_update_ms;


};

extern const AP_HAL::HAL& hal;
extern AP_PPK_FW ppk;
