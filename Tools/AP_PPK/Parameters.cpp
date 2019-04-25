#include "AP_PPK.h"

/*
 *  AP_PPK parameter definitions
 *
 */

#define GSCALAR(v, name, def) { ppk.g.v.vtype, name, Parameters::k_param_ ## v, &ppk.g.v, {def_value : def} }
#define ASCALAR(v, name, def) { ppk.aparm.v.vtype, name, Parameters::k_param_ ## v, (const void *)&ppk.aparm.v, {def_value : def} }
#define GGROUP(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, &ppk.g.v, {group_info : class::var_info} }
#define GOBJECT(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, (const void *)&ppk.v, {group_info : class::var_info} }
#define GOBJECTN(v, pname, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## pname, (const void *)&ppk.v, {group_info : class::var_info} }

const AP_Param::Info AP_PPK_FW::var_info[] = {
    // @Param: FORMAT_VERSION
    // @DisplayName: Eeprom format version number
    // @Description: This value is incremented when changes are made to the eeprom format
    // @User: Advanced
    GSCALAR(format_version,         "FORMAT_VERSION", 0),

    // GPS driver
    // @Group: GPS_
    // @Path: ../libraries/AP_GPS/AP_GPS.cpp
    GOBJECT(gps, "GPS_", AP_GPS),

    // @Group: SERIAL
    // @Path: ../libraries/AP_SerialManager/AP_SerialManager.cpp
    GOBJECT(serial_manager,  "SERIAL",   AP_SerialManager),

    // @Group: COMPASS_
    // @Path: ../libraries/AP_Compass/AP_Compass.cpp
//    GOBJECT(compass,         "COMPASS_",     Compass),

    AP_VAREND
};


void AP_PPK_FW::load_parameters(void)
{
    if (!g.format_version.load() ||
        g.format_version != Parameters::k_format_version) {

        // erase all parameters
        StorageManager::erase();
        AP_Param::erase_all();

        // save the current format version
        g.format_version.set_and_save(Parameters::k_format_version);
    }

    // Load all auto-loaded EEPROM variables
    AP_Param::load_all();
}
