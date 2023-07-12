#pragma once
#include <stdint.h>
#include <stdio.h>
#include <vehicle_msgs/IoniqControlCommand.h>
#include <vehicle_msgs/IoniqStatus.h>

#include <vector>

enum class IoniqSequenceState : uint8_t {
    kInit = 0,
    kBrakeReady = 1,
    kAccelReady = 2,
    kSteerReady = 3,
    kDriveReady = 4,
    kAutonomous = 5,
    kReqQuit = 6,
    kSteerQuit = 7,
    kAccelQuit = 8,
    kBrakeQuit = 9,
    kEnd = 10
};

enum class SteeringStateDATA : uint8_t {
    Reserved = 0x0,
    INIT = 0x1,
    READY = 0x2,
    STANDBY = 0x3,
    FIRST_ACTIVE = 0x4,
    LAST_ACTIVE = 0x5,
    ERORR = 0x6,
    ABORTED = 0x7,
    NOT_UES = 0xE,
    ERRO_INDICATOR = 0xF
};

struct IoniqStatus {
    double steering_speed;
    double cluster_odometer;
    double lateral_accel_speed;
    double brake_master_cylinder_pressur;
    // 0x0:Reserved/0x1:Steering still in initialization phase/0x2:Steering
    // ready, waits for PA command/0x3:Steering set in standby by
    // PA/0x4:Steering requested to go to first activation step/0x5:Steering
    // requested to go to final activation step/0x6:Steering went to error
    // internally/0x7:Steering aborted the automatic function/0xE:Not
    // Used/0xF:Error Indicator
    uint8_t steering_status;
    double longitudinal_accel_speed;
    double yaw_rate_sensor;
    // km/h
    double wheel_velocity_fr;
    double brake_active;
    // km/h
    double wheel_velocity_rl;
    // km/h
    double wheel_velocity_rr;
    // km/h
    double wheel_velocity_fl;
    double steering_angle;
    double steering_torque;
    double accel_pedal_position;
    // 0x0:P/0x1:B/0x5:D/0x6:N/0x7:R/0x9:Not Display at
    // Cluster/0xA:Reserved/0xB:Reserved/0xC:Manual Shift Mode on
    // Retromode/0xD:Reserved/0xE:Intermediate Position/0xF:Fault
    double gear_sel_disp;
    double f_mcu_torque;
    double r_mcu_torque;
};

struct IoniqControlCommnad {
    // bitpos: 24(3byte)
    // Length: 2
    // Default=0, VDModeOff=1, VDModeOn=2, RESERVED=3
    // Brake control enable
    uint8_t mode_act = 0;
    // 0~255(0~100%)
    double brk1 = 0;
    // 0~255(0~100%)
    uint8_t brk2_v = 0;
    double acc_aps = 0;
    uint16_t aps2 = 0;
    uint8_t f1 = 0;
    uint8_t f2 = 0;
    bool no = 0;
    bool nc = 0;
    // Accel control enable
    // 0x0: off
    // 0x1: on
    bool enable = 0;
    bool cmd2 = 0;
    // Gear lever position
    // 0x0:Initial value
    // 0x1:R/0x2:Nr
    // 0x3:Null
    // 0x4:Nd
    // 0x5:D
    // 0xF:Fault
    uint8_t shifter_rnd = 0;
    // Initial=0, Button On=1, Button Off=2, Fault=3
    uint8_t shifter_p = 0;
    // Initial=0, Button On=1, Button Off=2, Fault=3
    uint8_t r_button_status = 0;
    // Initial=0, Button On=1, Button Off=2, Fault=3
    uint8_t n_button_status = 0;
    // Initial=0, Button On=1, Button Off=2, Fault=3
    uint8_t d_button_status = 0;
    // Fault=0, Button On=1, Button Off=2, Initial=3
    uint8_t p_button_status_reversed = 0x3;
    // Fault=0, Button On=1, Button Off=2, Initial=3
    uint8_t r_button_status_reversed = 0x3;
    // Fault=0, Button On=1, Button Off=2, Initial=3
    uint8_t n_button_status_reversed = 0x3;
    // Fault=0, Button On=1, Button Off=2, Initial=3
    uint8_t d_button_status_reversed = 0x3;
    // 0x0:No display/0x1:PDW display/0x2:PA display/0x3:PDW & PA
    // display/0x4~0x5:Reserved/0x6:Not used/0x7:Error Indicator
    uint8_t steer_mod1 = 0;
    // 0x0:Default/0x1:Domestic/0x2:EU LHD/0x3:EU
    // RHD/0x4:Reserved/0x5:Reserved/0x6:Not used/0x7:Error Indicator
    uint8_t steer_mod2 = 0;
    // 0x0:PA/RSPA not applied/0x1:PA/RSPA applied/0x2:Not Used/0x3:Error
    // Indicator
    uint8_t steer_be = 0;
    // 0x0:Reserved/0x1:PA_Init/0x2:PA_NewStart/0x3:PA_Standby/0x4:PA_Active/0x5:PA_Angle_Control_Active/0x6:PA_Failure/0x7:PA_Abort/0x8~0xD:Reserved/0xE:Not
    // used/0xF:Error Indicator
    uint8_t steer_sta = 0;
    // 0x0:PA is not in Test Mode/0x1:PA is in Test Mode/0x2:Not used/0x3:Error
    // Indicator
    uint8_t steer_test_mod_sta = 0;
    // 0x0000~0xFFFF:Real Values
    double steer_str_angl_req = 0;
    // 0x0~0xE:Refer to Signal Description/0xF:Invalid
    uint8_t steer_opt_info = 0;
};

class IoniqCanParser {
   public:
    IoniqCanParser(){};
    ~IoniqCanParser(){};
    void parseData(const void* nbytes);
    void makeCmd(uint8_t* data);

    /**
     * @brief Convert IoniqStatus to vehicle_msgs::IoniqStatus
     *
     * @param in
     * @return vehicle_msgs::IoniqStatus
     */
    vehicle_msgs::IoniqStatus toMsg(const IoniqStatus& in);

    /**
     * @brief Convert IoniqStatus to vehicle_msgs::IoniqControlCommand
     *
     * @param in
     * @return vehicle_msgs::IoniqControlCommand
     */
    vehicle_msgs::IoniqControlCommand toMsg(const IoniqControlCommnad& in);

    /**
     * @brief Convert vehicle_msgs::IoniqStatus to IoniqStatus
     *
     * @param msg
     * @param out
     */
    void fromMsg(const vehicle_msgs::IoniqStatus& msg, IoniqStatus& out);

    /**
     * @brief Convert vehicle_msgs::IoniqControlCommand to IoniqControlCommnad
     *
     * @param msg
     * @param out
     */
    void fromMsg(const vehicle_msgs::IoniqControlCommand& msg, IoniqControlCommnad& out);

    void initCmd(IoniqControlCommnad& cmd);
    void printData(const IoniqStatus& data);
    void printCmd(const IoniqControlCommnad& cmd);
    const IoniqStatus& getStatus() const { return ioniq_status_; }
    const IoniqControlCommnad& getCmd() const { return ioniq_cmd_; }
    void setSteer(const double angle){
        ioniq_cmd_.steer_str_angl_req = angle;
    }
    void setAccel(const double accel) { ioniq_cmd_.acc_aps = accel; }
    void setBrake(const double brake) { ioniq_cmd_.brk1 = brake; }

   private:
    IoniqStatus ioniq_status_;
    IoniqControlCommnad ioniq_cmd_;
    IoniqSequenceState ioniq_sequence_state_ = IoniqSequenceState::kInit;
    bool longitudincal_control_enable_ = false;
    bool lateral_control_enable_ = false;
    // GAY1
    uint64_t decodeSasAngle(const void* nbytes);
    double rawToPhysSasAngle(uint64_t value);
    uint64_t decodeSasSpeed(const void* nbytes);
    double rawToPhysSasSpeed(uint64_t value);
    uint64_t decodeClusterOdometer(const void* nbytes);
    double rawToPhysClusterOdometer(uint64_t value);
    uint64_t decodeLateralAccelSpeed(const void* nbytes);
    double rawToPhysLateralAccelSpeed(uint64_t value);
    uint64_t decodeBrakeMasterCylinderPressur(const void* nbytes);
    double rawToPhysBrakeMasterCylinderPressur(uint64_t value);
    uint64_t decodeSteeringStatus(const void* nbytes);
    double rawToPhysSteeringStatus(uint64_t value);
    uint64_t decodeLongitudinalAccelSpeed(const void* nbytes);
    double rawToPhysLongitudinalAccelSpeed(uint64_t value);
    uint64_t decodeYawRateSensor(const void* nbytes);
    double rawToPhysYawRateSensor(uint64_t value);
    uint64_t decodeWheelVelocityFr(const void* nbytes);
    double rawToPhysWheelVelocityFr(uint64_t value);
    uint64_t decodeBrakeActive(const void* nbytes);
    double rawToPhysBrakeActive(uint64_t value);
    uint64_t decodeWheelVelocityRl(const void* nbytes);
    double rawToPhysWheelVelocityRl(uint64_t value);
    uint64_t decodeWheelVelocityRr(const void* nbytes);
    double rawToPhysWheelVelocityRr(uint64_t value);
    uint64_t decodeWheelVelocityFl(const void* nbytes);
    double rawToPhysWheelVelocityFl(uint64_t value);
    uint64_t decodeSteeringAngle(const void* nbytes);
    double rawToPhysSteeringAngle(uint64_t value);
    uint64_t decodeSteeringTq(const void* nbytes);
    double rawToPhysSteeringTq(uint64_t value);
    uint64_t decodeAccelPedalPosition(const void* nbytes);
    double rawToPhysAccelPedalPosition(uint64_t value);
    uint64_t decodeGearSelDisp(const void* nbytes);
    double rawToPhysGearSelDisp(uint64_t value);
    uint64_t decodeFMcuTorque(const void* nbytes);
    double rawToPhysFMcuTorque(uint64_t value);
    uint64_t decodeRMcuTorque(const void* nbytes);
    double rawToPhysRMcuTorque(uint64_t value);

    // ADCMD
    uint64_t decodeModeAct(const void* nbytes);
    double rawToPhysModeAct(uint64_t value);
    uint64_t decodeBrk1(const void* nbytes);
    double rawToPhysBrk1(uint64_t value);
    uint64_t decodeBrk2V(const void* nbytes);
    double rawToPhysBrk2V(uint64_t value);
    uint64_t decodeAccAps(const void* nbytes);
    double rawToPhysAccAps(uint64_t value);
    uint64_t decodeAps2(const void* nbytes);
    double rawToPhysAps2(uint64_t value);
    uint64_t decodeF1(const void* nbytes);
    double rawToPhysF1(uint64_t value);
    uint64_t decodeF2(const void* nbytes);
    double rawToPhysF2(uint64_t value);
    uint64_t decodeNo(const void* nbytes);
    double rawToPhysNo(uint64_t value);
    uint64_t decodeNc(const void* nbytes);
    double rawToPhysNc(uint64_t value);
    uint64_t decodeEnable(const void* nbytes);
    double rawToPhysEnable(uint64_t value);
    uint64_t decodeCmd2(const void* nbytes);
    double rawToPhys(uint64_t value);
    uint64_t decodeShifterRnd(const void* nbytes);
    double rawToPhysShifterRnd(uint64_t value);
    uint64_t decodeShifterP(const void* nbytes);
    double rawToPhysShifterP(uint64_t value);
    uint64_t decodeRButtonStatus(const void* nbytes);
    double rawToPhysRButtonStatus(uint64_t value);
    uint64_t decodeNButtonStatus(const void* nbytes);
    double rawToPhysNButtonStatus(uint64_t value);
    uint64_t decodeDButtonStatus(const void* nbytes);
    double rawToPhysDButtonStatus(uint64_t value);
    uint64_t decodePButtonStatusReversed(const void* nbytes);
    double rawToPhysPButtonStatusReversed(uint64_t value);
    uint64_t decodeRButtonStatusReversed(const void* nbytes);
    double rawToPhysRButtonStatusReversed(uint64_t value);
    uint64_t decodeNButtonStatusReversed(const void* nbytes);
    double rawToPhysNButtonStatusReversed(uint64_t value);
    uint64_t decodeDButtonStatusReversed(const void* nbytes);
    double rawToPhysDButtonStatusReversed(uint64_t value);

    uint64_t decodeSteerMod1(const void* nbytes);
    double rawToPhysSteerMod1(uint64_t value);
    uint64_t decodeSteerMod2(const void* nbytes);
    double rawToPhysSteerMod2(uint64_t value);
    uint64_t decodeSteerBe(const void* nbytes);
    double rawToPhysSteerBe(uint64_t value);
    uint64_t decodeSteerSta(const void* nbytes);
    double rawToPhysSteerSta(uint64_t value);
    uint64_t decodeSteerTestModSta(const void* nbytes);
    double rawToPhysSteerTestModSta(uint64_t value);
    uint64_t decodeSteerStrAnglReq(const void* nbytes);
    double rawToPhysSteerStrAnglReq(uint64_t value);
    uint64_t decodeSteerOptInfo(const void* nbytes);
    double rawToPhysSteerOptInfo(uint64_t value);
};
