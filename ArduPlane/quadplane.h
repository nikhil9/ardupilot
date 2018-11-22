#include <AP_Motors/AP_Motors.h>
#include <AC_PID/AC_PID.h>
#include <AC_AttitudeControl/AC_AttitudeControl_Multi.h> // Attitude control library
#include <AP_InertialNav/AP_InertialNav.h>
#include <AC_AttitudeControl/AC_PosControl.h>
#include <AC_WPNav/AC_WPNav.h>
#include <AC_WPNav/AC_Loiter.h>
#include <AC_Fence/AC_Fence.h>
#include <AC_Avoidance/AC_Avoid.h>
#include <AP_Proximity/AP_Proximity.h>

#define DATA_AUTOTUNE_INITIALISED           30
#define DATA_AUTOTUNE_OFF                   31
#define DATA_AUTOTUNE_RESTART               32
#define DATA_AUTOTUNE_SUCCESS               33
#define DATA_AUTOTUNE_FAILED                34
#define DATA_AUTOTUNE_REACHED_LIMIT         35
#define DATA_AUTOTUNE_PILOT_TESTING         36
#define DATA_AUTOTUNE_SAVEDGAINS            37

/*
  QuadPlane specific functionality
 */
class QuadPlane
{
public:
    friend class Plane;
    friend class AP_Tuning_Plane;
    friend class GCS_MAVLINK_Plane;
    friend class AP_AdvancedFailsafe_Plane;
    
    QuadPlane(AP_AHRS_NavEKF &_ahrs);

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];
    static const struct AP_Param::GroupInfo var_info2[];

    void control_run(void);
    void control_auto(const Location &loc);
    bool init_mode(void);
    bool setup(void);

    void vtol_position_controller(void);
    void setup_target_position(void);
    void takeoff_controller(void);
    void waypoint_controller(void);
    
    // update transition handling
    void update(void);

    // set motor arming
    void set_armed(bool armed);

    // is VTOL available?
    bool available(void) const {
        return initialised;
    }

    // is quadplane assisting?
    bool in_assisted_flight(void) const {
        return available() && assisted_flight;
    }

    /*
      return true if we are in a transition to fwd flight from hover
    */
    bool in_transition(void) const;

    /*
      return true if we are a tailsitter transitioning to VTOL flight
    */
    bool in_tailsitter_vtol_transition(void) const;
    
    bool handle_do_vtol_transition(enum MAV_VTOL_STATE state);

    bool do_vtol_takeoff(const AP_Mission::Mission_Command& cmd);
    bool do_vtol_land(const AP_Mission::Mission_Command& cmd);
    bool verify_vtol_takeoff(const AP_Mission::Mission_Command &cmd);
    bool verify_vtol_land(void);
    bool in_vtol_auto(void) const;
    bool in_vtol_mode(void) const;

    // vtol help for is_flying()
    bool is_flying(void);

    // return current throttle as a percentate
    uint8_t throttle_percentage(void) const {
        return last_throttle * 100;
    }

    // return desired forward throttle percentage
    int8_t forward_throttle_pct(void);        
    float get_weathervane_yaw_rate_cds(void);

    // see if we are flying from vtol point of view
    bool is_flying_vtol(void);

    // return true when tailsitter frame configured
    bool is_tailsitter(void) const;

    // return true when flying a tailsitter in VTOL
    bool tailsitter_active(void);
    
    // create outputs for tailsitters
    void tailsitter_output(void);

    // handle different tailsitter input types
    void tailsitter_check_input(void);
    
    // check if we have completed transition to fixed wing
    bool tailsitter_transition_fw_complete(void);

    // check if we have completed transition to vtol
    bool tailsitter_transition_vtol_complete(void) const;

    // account for surface speed scaling in hover
    void tailsitter_speed_scaling(void);
    
    // user initiated takeoff for guided mode
    bool do_user_takeoff(float takeoff_altitude);
    
    struct PACKED log_QControl_Tuning {
        LOG_PACKET_HEADER;
        uint64_t time_us;
        float    angle_boost;
        float    throttle_out;
        float    desired_alt;
        float    inav_alt;
        int16_t  desired_climb_rate;
        int16_t  climb_rate;
        float    dvx;
        float    dvy;
        float    dax;
        float    day;
        float    throttle_mix;
    };
        
private:
    AP_AHRS_NavEKF &ahrs;
    AP_Vehicle::MultiCopter aparm;

    AP_InertialNav_NavEKF inertial_nav{ahrs};

    AP_Int8 frame_class;
    AP_Int8 frame_type;
    
    AP_MotorsMulticopter *motors;
    const struct AP_Param::GroupInfo *motors_var_info;
    
    AC_AttitudeControl_Multi *attitude_control;
    AC_PosControl *pos_control;
    AC_WPNav *wp_nav;
    AC_Loiter *loiter_nav;
    
    // maximum vertical velocity the pilot may request
    AP_Int16 pilot_velocity_z_max;

    // vertical acceleration the pilot may request
    AP_Int16 pilot_accel_z;

    // check for quadplane assistance needed
    bool assistance_needed(float aspeed);

    // update transition handling
    void update_transition(void);

    // check for an EKF yaw reset
    void check_yaw_reset(void);
    
    // hold hover (for transition)
    void hold_hover(float target_climb_rate);    

    // hold stabilize (for transition)
    void hold_stabilize(float throttle_in);    

    // get pilot desired yaw rate in cd/s
    float get_pilot_input_yaw_rate_cds(void);

    // get overall desired yaw rate in cd/s
    float get_desired_yaw_rate_cds(void);
    
    // get desired climb rate in cm/s
    float get_pilot_desired_climb_rate_cms(void);

    // initialise throttle_wait when entering mode
    void init_throttle_wait();

    // use multicopter rate controller
    void multicopter_attitude_rate_update(float yaw_rate_cds);
    
    // main entry points for VTOL flight modes
    void init_stabilize(void);
    void control_stabilize(void);

    void check_attitude_relax(void);
    void init_hover(void);
    void control_hover(void);
    void run_rate_controller(void);

    void init_loiter(void);
    void init_land(void);
    void control_loiter(void);
    void check_land_complete(void);

    void init_qrtl(void);
    void control_qrtl(void);
    
    float assist_climb_rate_cms(void);

    // calculate desired yaw rate for assistance
    float desired_auto_yaw_rate_cds(void);

    bool should_relax(void);
    void motors_output(void);
    void Log_Write_QControl_Tuning();
    float landing_descent_rate_cms(float height_above_ground);
    
    // setup correct aux channels for frame class
    void setup_default_channels(uint8_t num_motors);

    void guided_start(void);
    void guided_update(void);

    void check_throttle_suppression(void);

    void run_z_controller(void);

    void setup_defaults(void);
    void setup_defaults_table(const struct defaults_struct *defaults, uint8_t count);

    // calculate a stopping distance for fixed-wing to vtol transitions
    float stopping_distance(void);
    
    AP_Int16 transition_time_ms;

    // transition deceleration, m/s/s
    AP_Float transition_decel;
    
    AP_Int16 rc_speed;

    // min and max PWM for throttle
    AP_Int16 thr_min_pwm;
    AP_Int16 thr_max_pwm;

    // speed below which quad assistance is given
    AP_Float assist_speed;

    // angular error at which quad assistance is given
    AP_Int8 assist_angle;
    uint32_t angle_error_start_ms;
    
    // maximum yaw rate in degrees/second
    AP_Float yaw_rate_max;

    // landing speed in cm/s
    AP_Int16 land_speed_cms;

    // QRTL start altitude, meters
    AP_Int16 qrtl_alt;
    
    // alt to switch to QLAND_FINAL
    AP_Float land_final_alt;
    AP_Float vel_forward_alt_cutoff;
    
    AP_Int8 enable;
    AP_Int8 transition_pitch_max;

    // control if a VTOL RTL will be used
    AP_Int8 rtl_mode;

    // control if a VTOL GUIDED will be used
    AP_Int8 guided_mode;

    // control ESC throttle calibration
    AP_Int8 esc_calibration;
    void run_esc_calibration(void);

    // ICEngine control on landing
    AP_Int8 land_icengine_cut;

    // HEARTBEAT mav_type override
    AP_Int8 mav_type;
    MAV_TYPE get_mav_type(void) const;
    
    // time we last got an EKF yaw reset
    uint32_t ekfYawReset_ms;

    struct {
        AP_Float gain;
        float integrator;
        uint32_t last_ms;
        int8_t last_pct;
    } vel_forward;

    struct {
        AP_Float gain;
        AP_Float min_roll;
        uint32_t last_pilot_input_ms;
        float last_output;
    } weathervane;
    
    bool initialised;
    
    // timer start for transition
    uint32_t transition_start_ms;

    Location last_auto_target;

    // last throttle value when active
    float last_throttle;

    // pitch when we enter loiter mode
    int32_t loiter_initial_pitch_cd;

    // when did we last run the attitude controller?
    uint32_t last_att_control_ms;

    // true if we have reached the airspeed threshold for transition
    enum {
        TRANSITION_AIRSPEED_WAIT,
        TRANSITION_TIMER,
        TRANSITION_ANGLE_WAIT_FW,
        TRANSITION_ANGLE_WAIT_VTOL,
        TRANSITION_DONE
    } transition_state;

    // true when waiting for pilot throttle
    bool throttle_wait:1;

    // true when quad is assisting a fixed wing mode
    bool assisted_flight:1;

    // true when in angle assist
    bool in_angle_assist:1;

    // are we in a guided takeoff?
    bool guided_takeoff:1;
    
    struct {
        // time when motors reached lower limit
        uint32_t lower_limit_start_ms;
        uint32_t land_start_ms;
        float vpos_start_m;
    } landing_detect;

    // time we last set the loiter target
    uint32_t last_loiter_ms;

    enum position_control_state {
        QPOS_POSITION1,
        QPOS_POSITION2,
        QPOS_LAND_DESCEND,
        QPOS_LAND_FINAL,
        QPOS_LAND_COMPLETE
    };
    struct {
        enum position_control_state state;
        float speed_scale;
        Vector2f target_velocity;
        float max_speed;
        Vector3f target;
        bool slow_descent:1;
    } poscontrol;

    struct {
        bool running;
        uint32_t start_ms;            // system time the motor test began
        uint32_t timeout_ms = 0;      // test will timeout this many milliseconds after the motor_test_start_ms
        uint8_t seq = 0;              // motor sequence number of motor being tested
        uint8_t throttle_type = 0;    // motor throttle type (0=throttle percentage, 1=PWM, 2=pilot throttle channel pass-through)
        uint16_t throttle_value = 0;  // throttle to be sent to motor, value depends upon it's type
        uint8_t motor_count;          // number of motors to cycle
    } motor_test;

    // time of last control log message
    uint32_t last_ctrl_log_ms;

    // types of tilt mechanisms
    enum {TILT_TYPE_CONTINUOUS=0,
          TILT_TYPE_BINARY=1,
          TILT_TYPE_VECTORED_YAW=2};
    
    // tiltrotor control variables
    struct {
        AP_Int16 tilt_mask;
        AP_Int16 max_rate_up_dps;
        AP_Int16 max_rate_down_dps;
        AP_Int8  max_angle_deg;
        AP_Int8  tilt_type;
        AP_Float tilt_yaw_angle;
        float current_tilt;
        float current_throttle;
        bool motors_active:1;
    } tilt;

    enum tailsitter_input {
        TAILSITTER_INPUT_MULTICOPTER = 0,
        TAILSITTER_INPUT_PLANE       = 1,
    };

    enum tailsitter_mask {
        TAILSITTER_MASK_AILERON  = 1,
        TAILSITTER_MASK_ELEVATOR = 2,
        TAILSITTER_MASK_THROTTLE = 4,
        TAILSITTER_MASK_RUDDER   = 8,
    };
    
    // tailsitter control variables
    struct {
        AP_Int8 transition_angle;
        AP_Int8 input_type;
        AP_Int8 input_mask;
        AP_Int8 input_mask_chan;
        AP_Float vectored_forward_gain;
        AP_Float vectored_hover_gain;
        AP_Float vectored_hover_power;
        AP_Float throttle_scale_max;
    } tailsitter;

    // the attitude view of the VTOL attitude controller
    AP_AHRS_View *ahrs_view;

    // time when motors were last active
    uint32_t last_motors_active_ms;

    // time when we last ran the vertical accel controller
    uint32_t last_pidz_active_ms;
    uint32_t last_pidz_init_ms;

    // time when we were last in a vtol control mode
    uint32_t last_vtol_mode_ms;
    
    void tiltrotor_slew(float tilt);
    void tiltrotor_binary_slew(bool forward);
    void tiltrotor_update(void);
    void tiltrotor_continuous_update(void);
    void tiltrotor_binary_update(void);
    void tiltrotor_vectored_yaw(void);
    void tilt_compensate_up(float *thrust, uint8_t num_motors);
    void tilt_compensate_down(float *thrust, uint8_t num_motors);
    void tilt_compensate(float *thrust, uint8_t num_motors);
    bool is_motor_tilting(uint8_t motor) const {
        return (((uint8_t)tilt.tilt_mask.get()) & (1U<<motor));
    }
    bool tiltrotor_fully_fwd(void);
    float tilt_max_change(bool up);

    void afs_terminate(void);
    bool guided_mode_enabled(void);

    // set altitude target to current altitude
    void set_alt_target_current(void);
    
    // adjust altitude target smoothly
    void adjust_alt_target(float target_cm);

    // additional options
    AP_Int32 options;
    enum {
        OPTION_LEVEL_TRANSITION=(1<<0),
        OPTION_ALLOW_FW_TAKEOFF=(1<<1),
        OPTION_ALLOW_FW_LAND=(1<<2),
    };

    /*
      return true if current mission item is a vtol takeoff
     */
    bool is_vtol_takeoff(uint16_t id) const;

    /*
      return true if current mission item is a vtol landing
     */
    bool is_vtol_land(uint16_t id) const;
    


    bool qautotune_init(bool ignore_checks);
    void qautotune_run();

    void qautotune_save_tuning_gains();
    void qautotune_stop();

    bool qautotune_start(bool ignore_checks);

    void qautotune_attitude_control();
    void qautotune_backup_gains_and_initialise();
    void qautotune_load_orig_gains();
    void qautotune_load_tuned_gains();
    void qautotune_load_intra_test_gains();
    void qautotune_load_twitch_gains();
    void qautotune_update_gcs(uint8_t message_id);
    bool qautotune_roll_enabled();
    bool qautotune_pitch_enabled();
    bool qautotune_yaw_enabled();
    void qautotune_twitching_test_rate(float rate, float rate_target, float &meas_rate_min, float &meas_rate_max);
    void qautotune_twitching_test_angle(float angle, float rate, float angle_target, float &meas_angle_min, float &meas_angle_max, float &meas_rate_min, float &meas_rate_max);
    void qautotune_twitching_measure_acceleration(float &rate_of_change, float rate_measurement, float &rate_measurement_max);
    void qautotune_updating_rate_d_up(float &tune_d, float tune_d_min, float tune_d_max, float tune_d_step_ratio, float &tune_p, float tune_p_min, float tune_p_max, float tune_p_step_ratio, float rate_target, float meas_rate_min, float meas_rate_max);
    void qautotune_updating_rate_d_down(float &tune_d, float tune_d_min, float tune_d_step_ratio, float &tune_p, float tune_p_min, float tune_p_max, float tune_p_step_ratio, float rate_target, float meas_rate_min, float meas_rate_max);
    void qautotune_updating_rate_p_up_d_down(float &tune_d, float tune_d_min, float tune_d_step_ratio, float &tune_p, float tune_p_min, float tune_p_max, float tune_p_step_ratio, float rate_target, float meas_rate_min, float meas_rate_max);
    void qautotune_updating_angle_p_down(float &tune_p, float tune_p_min, float tune_p_step_ratio, float angle_target, float meas_angle_max, float meas_rate_min, float meas_rate_max);
    void qautotune_updating_angle_p_up(float &tune_p, float tune_p_max, float tune_p_step_ratio, float angle_target, float meas_angle_max, float meas_rate_min, float meas_rate_max);
    void qautotune_get_poshold_attitude(float &roll_cd, float &pitch_cd, float &yaw_cd);


    void Log_Write_AutoTune(uint8_t axis, uint8_t tune_step, float meas_target, float meas_min, float meas_max, float new_gain_rp, float new_gain_rd, float new_gain_sp, float new_ddt);
    void Log_Write_AutoTuneDetails(float angle_cd, float rate_cds);
    void Log_Write_Event(uint8_t id);


    void qautotune_send_step_string();
    const char *qautotune_level_issue_string() const;
    const char * qautotune_type_string() const;
    void qautotune_announce_state_to_gcs();
    void qautotune_do_gcs_announcements();

    enum LEVEL_ISSUE {
        LEVEL_ISSUE_NONE,
        LEVEL_ISSUE_ANGLE_ROLL,
        LEVEL_ISSUE_ANGLE_PITCH,
        LEVEL_ISSUE_ANGLE_YAW,
        LEVEL_ISSUE_RATE_ROLL,
        LEVEL_ISSUE_RATE_PITCH,
        LEVEL_ISSUE_RATE_YAW,
    };
    bool qautotune_check_level(const enum LEVEL_ISSUE issue, const float current, const float maximum);
    bool qautotune_currently_level();

    // autotune modes (high level states)
    enum TuneMode {
        UNINITIALISED = 0,        // autotune has never been run
        TUNING = 1,               // autotune is testing gains
        SUCCESS = 2,              // tuning has completed, user is flight testing the new gains
        FAILED = 3,               // tuning has failed, user is flying on original gains
    };

    // steps performed while in the tuning mode
    enum StepType {
        WAITING_FOR_LEVEL = 0,    // autotune is waiting for vehicle to return to level before beginning the next twitch
        TWITCHING = 1,            // autotune has begun a twitch and is watching the resulting vehicle movement
        UPDATE_GAINS = 2          // autotune has completed a twitch and is updating the gains based on the results
    };

    // things that can be tuned
    enum AxisType {
        ROLL = 0,                 // roll axis is being tuned (either angle or rate)
        PITCH = 1,                // pitch axis is being tuned (either angle or rate)
        YAW = 2,                  // pitch axis is being tuned (either angle or rate)
    };

    // mini steps performed while in Tuning mode, Testing step
    enum TuneType {
        RD_UP = 0,                // rate D is being tuned up
        RD_DOWN = 1,              // rate D is being tuned down
        RP_UP = 2,                // rate P is being tuned up
        SP_DOWN = 3,              // angle P is being tuned down
        SP_UP = 4                 // angle P is being tuned up
    };

    TuneMode mode                : 2;    // see TuneMode for what modes are allowed
    bool     pilot_override      : 1;    // true = pilot is overriding controls so we suspend tuning temporarily
    AxisType axis                : 2;    // see AxisType for which things can be tuned
    bool     positive_direction  : 1;    // false = tuning in negative direction (i.e. left for roll), true = positive direction (i.e. right for roll)
    StepType step                : 2;    // see StepType for what steps are performed
    TuneType tune_type           : 3;    // see TuneType
    bool     ignore_next         : 1;    // true = ignore the next test
    bool     twitch_first_iter   : 1;    // true on first iteration of a twitch (used to signal we must step the attitude or rate target)
    bool     use_poshold         : 1;    // true = enable position hold
    bool     have_position       : 1;    // true = start_position is value
    Vector3f start_position;

// variables
    uint32_t override_time;                         // the last time the pilot overrode the controls
    float    test_rate_min;                         // the minimum angular rate achieved during TESTING_RATE step
    float    test_rate_max;                         // the maximum angular rate achieved during TESTING_RATE step
    float    test_angle_min;                        // the minimum angle achieved during TESTING_ANGLE step
    float    test_angle_max;                        // the maximum angle achieved during TESTING_ANGLE step
    uint32_t step_start_time;                       // start time of current tuning step (used for timeout checks)
    uint32_t step_stop_time;                        // start time of current tuning step (used for timeout checks)
    int8_t   counter;                               // counter for tuning gains
    float    target_rate, start_rate;               // target and start rate
    float    target_angle, start_angle;             // target and start angles
    float    desired_yaw;                           // yaw heading during tune
    float    rate_max, test_accel_max;              // maximum acceleration variables

    LowPassFilterFloat  rotation_rate_filt;         // filtered rotation rate in radians/second

// backup of currently being tuned parameter values
    float    orig_roll_rp = 0, orig_roll_ri, orig_roll_rd, orig_roll_sp, orig_roll_accel;
    float    orig_pitch_rp = 0, orig_pitch_ri, orig_pitch_rd, orig_pitch_sp, orig_pitch_accel;
    float    orig_yaw_rp = 0, orig_yaw_ri, orig_yaw_rd, orig_yaw_rLPF, orig_yaw_sp, orig_yaw_accel;
    bool     orig_bf_feedforward;

// currently being tuned parameter values
    float    tune_roll_rp, tune_roll_rd, tune_roll_sp, tune_roll_accel;
    float    tune_pitch_rp, tune_pitch_rd, tune_pitch_sp, tune_pitch_accel;
    float    tune_yaw_rp, tune_yaw_rLPF, tune_yaw_sp, tune_yaw_accel;

    uint32_t announce_time;
    float lean_angle;
    float rotation_rate;
    float qautotune_roll_cd, qautotune_pitch_cd;

    struct {
        LEVEL_ISSUE issue{LEVEL_ISSUE_NONE};
        float maximum;
        float current;
    } level_problem;

    AP_Int8                 autotune_axis_bitmask;
    AP_Float                autotune_aggressiveness;
    AP_Float                autotune_min_d;

public:
    void motor_test_output();
    MAV_RESULT mavlink_motor_test_start(mavlink_channel_t chan, uint8_t motor_seq, uint8_t throttle_type,
                                        uint16_t throttle_value, float timeout_sec,
                                        uint8_t motor_count);
private:
    void motor_test_stop();
};
