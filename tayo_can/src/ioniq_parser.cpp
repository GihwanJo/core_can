#include "tayo_can/ioniq_parser.h"

uint64_t IoniqCanParser::decodeSasAngle(const void* nbytes) {
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(nbytes);
    data >>= 0ull;
    data &= 65535ull;
    return data;
}
double IoniqCanParser::rawToPhysSasAngle(uint64_t value) {
    int16_t tmp = 0;
    tmp = tmp | value;
    return tmp * 0.1;
    return static_cast<int16_t>(value * 0.1 + 0);
}
uint64_t IoniqCanParser::decodeSasSpeed(const void* nbytes) {
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(nbytes);
    data >>= 16ull;
    data &= 255ull;
    return data;
}
double IoniqCanParser::rawToPhysSasSpeed(uint64_t value) {
    return value * 4 + 0;
}
uint64_t IoniqCanParser::decodeClusterOdometer(const void* nbytes) {
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(nbytes);
    data >>= 24ull;
    data &= 16777215ull;
    return data;
}
double IoniqCanParser::rawToPhysClusterOdometer(uint64_t value) {
    return value * 0.1 + 0;
}
uint64_t IoniqCanParser::decodeLateralAccelSpeed(const void* nbytes) {
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(nbytes);
    data >>= 48ull;
    data &= 65535ull;
    return data;
}
double IoniqCanParser::rawToPhysLateralAccelSpeed(uint64_t value) {
    return value * 0.000127465 + -4.17677;
}
uint64_t IoniqCanParser::decodeBrakeMasterCylinderPressur(const void* nbytes) {
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(
        &reinterpret_cast<const uint8_t*>(nbytes)[8]);
    data >>= 0ull;
    data &= 4095ull;
    return data;
}
double IoniqCanParser::rawToPhysBrakeMasterCylinderPressur(uint64_t value) {
    return value * 0.1 + 0;
}
uint64_t IoniqCanParser::decodeSteeringStatus(const void* nbytes) {
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(
        &reinterpret_cast<const uint8_t*>(nbytes)[8]);
    data >>= 12ull;
    data &= 15ull;
    return data;
}
double IoniqCanParser::rawToPhysSteeringStatus(uint64_t value) {
    return value * 1 + 0;
}
uint64_t IoniqCanParser::decodeLongitudinalAccelSpeed(const void* nbytes) {
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(
        &reinterpret_cast<const uint8_t*>(nbytes)[8]);
    data >>= 16ull;
    data &= 65535ull;
    return data;
}
double IoniqCanParser::rawToPhysLongitudinalAccelSpeed(uint64_t value) {
    return value * 0.000127465 + -4.17677;
}
uint64_t IoniqCanParser::decodeYawRateSensor(const void* nbytes) {
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(
        &reinterpret_cast<const uint8_t*>(nbytes)[8]);
    data >>= 32ull;
    data &= 65535ull;
    return data;
}
double IoniqCanParser::rawToPhysYawRateSensor(uint64_t value) {
    return value * 0.005 + -163.84;
}
uint64_t IoniqCanParser::decodeWheelVelocityFr(const void* nbytes) {
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(
        &reinterpret_cast<const uint8_t*>(nbytes)[8]);
    data >>= 48ull;
    data &= 16383ull;
    return data;
}
double IoniqCanParser::rawToPhysWheelVelocityFr(uint64_t value) {
    return value * 0.03125 + 0;
}
uint64_t IoniqCanParser::decodeBrakeActive(const void* nbytes) {
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(
        &reinterpret_cast<const uint8_t*>(nbytes)[8]);
    data >>= 62ull;
    data &= 3ull;
    return data;
}
double IoniqCanParser::rawToPhysBrakeActive(uint64_t value) {
    return value * 1 + 0;
}
uint64_t IoniqCanParser::decodeWheelVelocityRl(const void* nbytes) {
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(
        &reinterpret_cast<const uint8_t*>(nbytes)[16]);
    data >>= 0ull;
    data &= 16383ull;
    return data;
}
double IoniqCanParser::rawToPhysWheelVelocityRl(uint64_t value) {
    return value * 0.03125 + 0;
}
uint64_t IoniqCanParser::decodeWheelVelocityRr(const void* nbytes) {
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(
        &reinterpret_cast<const uint8_t*>(nbytes)[16]);
    data >>= 16ull;
    data &= 16383ull;
    return data;
}
double IoniqCanParser::rawToPhysWheelVelocityRr(uint64_t value) {
    return value * 0.03125 + 0;
}
uint64_t IoniqCanParser::decodeWheelVelocityFl(const void* nbytes) {
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(
        &reinterpret_cast<const uint8_t*>(nbytes)[16]);
    data >>= 32ull;
    data &= 16383ull;
    return data;
}
double IoniqCanParser::rawToPhysWheelVelocityFl(uint64_t value) {
    return value * 0.03125 + 0;
}
uint64_t IoniqCanParser::decodeSteeringAngle(const void* nbytes) {
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(
        &reinterpret_cast<const uint8_t*>(nbytes)[16]);
    data >>= 48ull;
    data &= 65535ull;
    if (data & 18446744073709518848ull) {
        data |= 18446744073709518848ull;
    }
    return data;
    return data;
}
double IoniqCanParser::rawToPhysSteeringAngle(uint64_t value) {
    return value * 0.1 + 0;
}
uint64_t IoniqCanParser::decodeSteeringTq(const void* nbytes) {
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(
        &reinterpret_cast<const uint8_t*>(nbytes)[24]);
    data >>= 0ull;
    data &= 8191ull;
    return data;
}
double IoniqCanParser::rawToPhysSteeringTq(uint64_t value) {
    return value * 0.005 + -20.48;
}
uint64_t IoniqCanParser::decodeAccelPedalPosition(const void* nbytes) {
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(
        &reinterpret_cast<const uint8_t*>(nbytes)[24]);
    data >>= 16ull;
    data &= 255ull;
    return data;
}
double IoniqCanParser::rawToPhysAccelPedalPosition(uint64_t value) {
    return value * 0.392157 + 0;
}
uint64_t IoniqCanParser::decodeGearSelDisp(const void* nbytes) {
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(
        &reinterpret_cast<const uint8_t*>(nbytes)[24]);
    data >>= 24ull;
    data &= 15ull;
    return data;
}
double IoniqCanParser::rawToPhysGearSelDisp(uint64_t value) {
    return value * 1 + 0;
}

uint64_t IoniqCanParser::decodeFMcuTorque(const void* nbytes) {
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(
        &reinterpret_cast<const uint8_t*>(nbytes)[24]);
    data >>= 32ull;
    data &= 16383ull;
    if (data & 18446744073709543424ull) {
        data |= 18446744073709543424ull;
    }
    return data;
}
double IoniqCanParser::rawToPhysFMcuTorque(uint64_t value) {
    uint32_t tmp = value;
    int32_t v =0;
    v = v | tmp;
    return v * 0.125 + 0;
}
uint64_t IoniqCanParser::decodeRMcuTorque(const void* nbytes) {
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(
        &reinterpret_cast<const uint8_t*>(nbytes)[24]);
    data >>= 48ull;
    data &= 16383ull;
    if (data & 18446744073709543424ull) {
        data |= 18446744073709543424ull;
    }
    return data;
}
double IoniqCanParser::rawToPhysRMcuTorque(uint64_t value) {
    uint32_t tmp = value;
    int32_t v =0;
    v = v | tmp;
    return v * 0.125 + 0;
}

uint64_t IoniqCanParser::decodeModeAct(const void* nbytes) {
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(nbytes);
    data >>= 24ull;
    data &= 3ull;
    return data;
}
double IoniqCanParser::rawToPhysModeAct(uint64_t value) {
    return value * 1 + 0;
}
uint64_t IoniqCanParser::decodeBrk1(const void* nbytes) {
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(nbytes);
    data >>= 32ull;
    data &= 255ull;
    return data;
}
double IoniqCanParser::rawToPhysBrk1(uint64_t value) {
    return value * 0.390625 + 0;
}
uint64_t IoniqCanParser::decodeBrk2V(const void* nbytes) {
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(nbytes);
    data >>= 40ull;
    data &= 255ull;
    return data;
}
double IoniqCanParser::rawToPhysBrk2V(uint64_t value) {
    return value * 0.390625 + 0;
}
uint64_t IoniqCanParser::decodeAccAps(const void* nbytes) {
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(nbytes);
    data >>= 48ull;
    data &= 65535ull;
    return data;
}
double IoniqCanParser::rawToPhysAccAps(uint64_t value) {
    return value * 0.669841 + 615;
}
uint64_t IoniqCanParser::decodeAps2(const void* nbytes) {
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(
        &reinterpret_cast<const uint8_t*>(nbytes)[8]);
    data >>= 0ull;
    data &= 65535ull;
    return data;
}
double IoniqCanParser::rawToPhysAps2(uint64_t value) {
    return value * 0.334799 + 308;
}
uint64_t IoniqCanParser::decodeF1(const void* nbytes) {
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(
        &reinterpret_cast<const uint8_t*>(nbytes)[8]);
    data >>= 16ull;
    data &= 15ull;
    return data;
}
double IoniqCanParser::rawToPhysF1(uint64_t value) { return value * 1 + 0; }
uint64_t IoniqCanParser::decodeF2(const void* nbytes) {
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(
        &reinterpret_cast<const uint8_t*>(nbytes)[8]);
    data >>= 20ull;
    data &= 15ull;
    return data;
}
double IoniqCanParser::rawToPhysF2(uint64_t value) { return value * 1 + 0; }
uint64_t IoniqCanParser::decodeNo(const void* nbytes) {
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(
        &reinterpret_cast<const uint8_t*>(nbytes)[8]);
    data >>= 24ull;
    data &= 1ull;
    return data;
}
double IoniqCanParser::rawToPhysNo(uint64_t value) { return value * 1 + 0; }
uint64_t IoniqCanParser::decodeNc(const void* nbytes) {
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(
        &reinterpret_cast<const uint8_t*>(nbytes)[8]);
    data >>= 25ull;
    data &= 1ull;
    return data;
}
double IoniqCanParser::rawToPhysNc(uint64_t value) { return value * 1 + 0; }
uint64_t IoniqCanParser::decodeEnable(const void* nbytes) {
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(
        &reinterpret_cast<const uint8_t*>(nbytes)[8]);
    data >>= 26ull;
    data &= 1ull;
    return data;
}
double IoniqCanParser::rawToPhysEnable(uint64_t value) { return value * 1 + 0; }
uint64_t IoniqCanParser::decodeCmd2(const void* nbytes) {
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(
        &reinterpret_cast<const uint8_t*>(nbytes)[8]);
    data >>= 27ull;
    data &= 1ull;
    return data;
}
double IoniqCanParser::rawToPhys(uint64_t value) { return value * 1 + 0; }
uint64_t IoniqCanParser::decodeShifterRnd(const void* nbytes) {
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(
        &reinterpret_cast<const uint8_t*>(nbytes)[8]);
    data >>= 28ull;
    data &= 15ull;
    return data;
}
double IoniqCanParser::rawToPhysShifterRnd(uint64_t value) {
    return value * 1 + 0;
}
uint64_t IoniqCanParser::decodeShifterP(const void* nbytes) {
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(
        &reinterpret_cast<const uint8_t*>(nbytes)[8]);
    data >>= 32ull;
    data &= 3ull;
    return data;
}
double IoniqCanParser::rawToPhysShifterP(uint64_t value) {
    return value * 1 + 0;
}
uint64_t IoniqCanParser::decodeRButtonStatus(const void* nbytes) {
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(
        &reinterpret_cast<const uint8_t*>(nbytes)[8]);
    data >>= 34ull;
    data &= 3ull;
    return data;
}
double IoniqCanParser::rawToPhysRButtonStatus(uint64_t value) {
    return value * 1 + 0;
}
uint64_t IoniqCanParser::decodeNButtonStatus(const void* nbytes) {
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(
        &reinterpret_cast<const uint8_t*>(nbytes)[8]);
    data >>= 36ull;
    data &= 3ull;
    return data;
}
double IoniqCanParser::rawToPhysNButtonStatus(uint64_t value) {
    return value * 1 + 0;
}
uint64_t IoniqCanParser::decodeDButtonStatus(const void* nbytes) {
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(
        &reinterpret_cast<const uint8_t*>(nbytes)[8]);
    data >>= 38ull;
    data &= 3ull;
    return data;
}
double IoniqCanParser::rawToPhysDButtonStatus(uint64_t value) {
    return value * 1 + 0;
}
uint64_t IoniqCanParser::decodePButtonStatusReversed(const void* nbytes) {
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(
        &reinterpret_cast<const uint8_t*>(nbytes)[8]);
    data >>= 40ull;
    data &= 3ull;
    return data;
}
double IoniqCanParser::rawToPhysPButtonStatusReversed(uint64_t value) {
    return value * 1 + 0;
}
uint64_t IoniqCanParser::decodeRButtonStatusReversed(const void* nbytes) {
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(
        &reinterpret_cast<const uint8_t*>(nbytes)[8]);
    data >>= 42ull;
    data &= 3ull;
    return data;
}
double IoniqCanParser::rawToPhysRButtonStatusReversed(uint64_t value) {
    return value * 1 + 0;
}
uint64_t IoniqCanParser::decodeNButtonStatusReversed(const void* nbytes) {
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(
        &reinterpret_cast<const uint8_t*>(nbytes)[8]);
    data >>= 44ull;
    data &= 3ull;
    return data;
}
double IoniqCanParser::rawToPhysNButtonStatusReversed(uint64_t value) {
    return value * 1 + 0;
}
uint64_t IoniqCanParser::decodeDButtonStatusReversed(const void* nbytes) {
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(
        &reinterpret_cast<const uint8_t*>(nbytes)[8]);
    data >>= 46ull;
    data &= 3ull;
    return data;
}
double IoniqCanParser::rawToPhysDButtonStatusReversed(uint64_t value) {
    return value * 1 + 0;
}

uint64_t IoniqCanParser::decodeSteerMod1(const void* nbytes) {
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(
        &reinterpret_cast<const uint8_t*>(nbytes)[8]);
    data >>= 48ull;
    data &= 7ull;
    return data;
}
double IoniqCanParser::rawToPhysSteerMod1(uint64_t value) {
    return value * 1 + 0;
}
uint64_t IoniqCanParser::decodeSteerMod2(const void* nbytes) {
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(
        &reinterpret_cast<const uint8_t*>(nbytes)[8]);
    data >>= 51ull;
    data &= 7ull;
    return data;
}
double IoniqCanParser::rawToPhysSteerMod2(uint64_t value) {
    return value * 1 + 0;
}
uint64_t IoniqCanParser::decodeSteerBe(const void* nbytes) {
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(
        &reinterpret_cast<const uint8_t*>(nbytes)[8]);
    data >>= 54ull;
    data &= 3ull;
    return data;
}
double IoniqCanParser::rawToPhysSteerBe(uint64_t value) {
    return value * 1 + 0;
}
uint64_t IoniqCanParser::decodeSteerSta(const void* nbytes) {
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(
        &reinterpret_cast<const uint8_t*>(nbytes)[8]);
    data >>= 56ull;
    data &= 15ull;
    return data;
}
double IoniqCanParser::rawToPhysSteerSta(uint64_t value) {
    return value * 1 + 0;
}
uint64_t IoniqCanParser::decodeSteerTestModSta(const void* nbytes) {
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(
        &reinterpret_cast<const uint8_t*>(nbytes)[8]);
    data >>= 60ull;
    data &= 3ull;
    return data;
}
double IoniqCanParser::rawToPhysSteerTestModSta(uint64_t value) {
    return value * 1 + 0;
}
uint64_t IoniqCanParser::decodeSteerStrAnglReq(const void* nbytes) {
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(
        &reinterpret_cast<const uint8_t*>(nbytes)[16]);
    data >>= 0ull;
    data &= 65535ull;
    if (data & 18446744073709518848ull) {
        data |= 18446744073709518848ull;
    }
    return data;
    return data;
}
double IoniqCanParser::rawToPhysSteerStrAnglReq(uint64_t value) {
    return value * 0.1 + 0;
}
uint64_t IoniqCanParser::decodeSteerOptInfo(const void* nbytes) {
    uint64_t data;
    data = *reinterpret_cast<const uint64_t*>(
        &reinterpret_cast<const uint8_t*>(nbytes)[16]);
    data >>= 16ull;
    data &= 15ull;
    return data;
}
double IoniqCanParser::rawToPhysSteerOptInfo(uint64_t value) {
    return value * 1 + 0;
}

// ADCMD_ModeAct
// 브레이크 값 넣을떄 enable 해야함
// [3] 2bit

// Brk1
// 페달 브레이크
// [4] 8bit

// CMD_Enable
// 이걸 해야 기어 돌리는거(ShifterRND) 됨
// [11].3 1bit

// ShifterRND
// 기어 돌리는것
// R 쪽으로 2가지, D쪽으로 두가지

// StatusRevers들은 3(initail)을 다 넣어줘야 오류가 안남
// [16][17] 16bit

// steer req 단독으로 값 안들어감

// ADSteer_Sta
// PA(Parking Assit)를 사용해서 핸들돌림
// 한번에 steer를 많이 돌리게 값을 넣으면 오류나면서 핸들이 더이상 제어가 안됨,
// 하지만 돌리는 속도 자체는 개빠름 순서대로 넣으면 개빨리 돌릴 수 있을거 같음
// 확인 필요 1->2->3->4->5 순서대로 넣어야 할듯 3->4->5도 가능
// Gway_Steering_Status가 똑같이 1->2->3->4->5 순서대로 움직임
// 입력하고 Gway_Steering_status가 변한걸 확인하고 다음 can을 넣으면 될듯
// 7번 오류에서 다시 돌아가는 것도 찾아야함

// ADCMD_AccAps
// 정지상태(i-PEDAL, D, 파킹브레이크)에서 가속도 0x1AB(427)까지 들어감
// can 값으로는 901.0222가 나옴 도대체 무슨 기준으로 값이 들어가는걸까????
// APS2는 도대체 뭐냐?

// -드라이브모드(에코, 노말, 스포츠)에 따라서 같은 가속도를 입력해도 차량의
// 가속력이 다름

void IoniqCanParser::printCmd(const IoniqControlCommnad& cmd) {
    printf("cmd.mode_act : %d\n", cmd.mode_act);
    printf("cmd.brk1 : %f\n", cmd.brk1);
    printf("cmd.acc_aps : %f\n", cmd.acc_aps);
    printf("cmd.shifter_rnd : %d\n", cmd.shifter_rnd);
    printf("cmd.shifter_p : %d\n", cmd.shifter_p);
    printf("cmd.enable : %d\n", cmd.enable);
    printf("cmd.p_button_status_reversed : %d\n", cmd.p_button_status_reversed);
    printf("cmd.r_button_status_reversed : %d\n", cmd.r_button_status_reversed);
    printf("cmd.n_button_status_reversed : %d\n", cmd.n_button_status_reversed);
    printf("cmd.d_button_status_reversed : %d\n", cmd.d_button_status_reversed);
    printf("cmd.steer_mod1 : %d\n", cmd.steer_mod1);
    printf("cmd.steer_mod2 : %d\n", cmd.steer_mod2);
    printf("cmd.steer_be : %d\n", cmd.steer_be);
    printf("cmd.steer_sta : %d\n", cmd.steer_sta);
    printf("cmd.steer_str_angl_req : %4f\n", cmd.steer_str_angl_req);
}

void IoniqCanParser::makeCmd(uint8_t* data) {
    initCmd(ioniq_cmd_);

    for (int i = 0; i < 32; ++i) data[i] = 0;

    data[3] |= ioniq_cmd_.mode_act;
    constexpr double brake_min = 0.0;
    constexpr double brake_max = 99.9;
    if (ioniq_cmd_.brk1 > brake_max) ioniq_cmd_.brk1 = brake_max;
    if (ioniq_cmd_.brk1 < brake_min) ioniq_cmd_.brk1 = brake_min;
    uint8_t tmp = static_cast<uint8_t>(ioniq_cmd_.brk1 * 2.56);
    data[4] |= tmp;

    constexpr double accel_min = 0.0;
    constexpr double accel_max = 99.9;
    if(ioniq_cmd_.acc_aps > accel_max) ioniq_cmd_.acc_aps = accel_max;
    if(ioniq_cmd_.acc_aps < accel_min) ioniq_cmd_.acc_aps = accel_min;

    constexpr double acc_offset = 22.3875;
    constexpr double acc_factor = 1.0 / 0.2985;
    uint16_t tmp_uint16 =
        static_cast<uint16_t>((ioniq_cmd_.acc_aps + acc_offset) * acc_factor);
    uint16_t upper, lower;
    lower = (tmp_uint16 & 0xFF);
    upper = (tmp_uint16 >> 8) & 0xFF;
    data[6] = lower;
    data[7] = upper;

    data[11] |= (ioniq_cmd_.shifter_rnd << 4) & 0b11110000;

    data[11] |= ioniq_cmd_.enable << 2;

    data[12] |= ioniq_cmd_.shifter_p;
    data[12] |= ioniq_cmd_.r_button_status << 2;
    data[12] |= ioniq_cmd_.n_button_status << 4;
    data[12] |= ioniq_cmd_.d_button_status << 6;

    data[13] |= ioniq_cmd_.p_button_status_reversed;
    data[13] |= ioniq_cmd_.r_button_status_reversed << 2;
    data[13] |= ioniq_cmd_.n_button_status_reversed << 4;
    data[13] |= ioniq_cmd_.d_button_status_reversed << 6;

    data[14] |= ioniq_cmd_.steer_mod1 & 0b00000111;
    data[14] |= (ioniq_cmd_.steer_mod2 << 3) & 0b00111000;
    data[14] |= (ioniq_cmd_.steer_be << 6) & 0b11000000;

    data[15] |= ioniq_cmd_.steer_sta & 0b00001111;
    data[15] |= (ioniq_cmd_.steer_test_mod_sta << 4) & 0b00110000;

    constexpr double steer_max = 480.0;
    constexpr double steer_min = -480.0;
    if (ioniq_cmd_.steer_str_angl_req > steer_max)
        ioniq_cmd_.steer_str_angl_req = steer_max;
    if (ioniq_cmd_.steer_str_angl_req < steer_min)
        ioniq_cmd_.steer_str_angl_req = steer_min;

    tmp_uint16 = static_cast<uint16_t>(ioniq_cmd_.steer_str_angl_req * 10);
    lower = (tmp_uint16 & 0xFF);
    upper = (tmp_uint16 >> 8) & 0xFF;
    data[16] = lower;
    data[17] = upper;
}

void IoniqCanParser::initCmd(IoniqControlCommnad& cmd) {
    switch (ioniq_sequence_state_) {
        case IoniqSequenceState::kInit: {
            cmd.mode_act = 2;
            if (ioniq_status_.gear_sel_disp == 0) {
                cmd.brk1 = 64;
                if (ioniq_status_.brake_master_cylinder_pressur > 25)
                    ioniq_sequence_state_ = IoniqSequenceState::kBrakeReady;
            } else if (ioniq_status_.gear_sel_disp == 5) {
                ioniq_sequence_state_ = IoniqSequenceState::kBrakeReady;
            }
            break;
        }
        case IoniqSequenceState::kBrakeReady: {
            cmd.mode_act = 2;
            cmd.acc_aps = 0;
            if (ioniq_status_.gear_sel_disp == 0) {
                cmd.brk1 = 64;
                if (ioniq_status_.gear_sel_disp != 0) {
                    cmd.shifter_p = 1;
                    cmd.enable = 1;
                } else {
                    cmd.shifter_p = 2;
                    cmd.enable = 1;
                    ioniq_sequence_state_ = IoniqSequenceState::kAccelReady;
                }
            } else if (ioniq_status_.gear_sel_disp == 5) {
                cmd.brk1 = 0;
                cmd.shifter_p = 0;
                cmd.enable = 1;
                ioniq_sequence_state_ = IoniqSequenceState::kAccelReady;
            }
            break;
        }
        case IoniqSequenceState::kAccelReady: {
            cmd.steer_be = 0x1;
            cmd.steer_mod1 = 0x2;
            cmd.steer_mod2 = 0x1;
            if (ioniq_status_.steering_status == 0x00)
                cmd.steer_sta = 0x01;
            else if (ioniq_status_.steering_status == 0x01)
                cmd.steer_sta = 0x03;
            else if (ioniq_status_.steering_status == 0x02)
                cmd.steer_sta = 0x03;
            else if (ioniq_status_.steering_status == 0x03) {
                cmd.steer_sta = 0x04;
                cmd.steer_str_angl_req = 0;
            } else if (ioniq_status_.steering_status == 0x04) {
                cmd.steer_sta = 0x05;
                cmd.steer_str_angl_req = 0;
                if (ioniq_status_.gear_sel_disp == 0) {
                    cmd.brk1 = 64;
                } else if (ioniq_status_.gear_sel_disp == 5)
                    cmd.brk1 = 0;

                ioniq_sequence_state_ = IoniqSequenceState::kDriveReady;
            } else {
                cmd.steer_sta = 0x03;
            }
            break;
        }
        case IoniqSequenceState::kDriveReady: {
            if (ioniq_status_.gear_sel_disp != 5) {
                cmd.shifter_rnd = 5;
                cmd.brk1 = 64;
            } else if (ioniq_status_.gear_sel_disp == 5) {
                cmd.shifter_rnd = 3;
                cmd.brk1 = 0;
                ioniq_sequence_state_ = IoniqSequenceState::kAutonomous;
            }
            break;
        }
        case IoniqSequenceState::kAutonomous: {
            break;
        }
        case IoniqSequenceState::kReqQuit: {
            cmd.steer_str_angl_req = 0;
            cmd.steer_sta = 3;
            cmd.steer_be = 0;
            cmd.steer_mod1 = 0;
            cmd.steer_mod2 = 0;
            ioniq_sequence_state_ = IoniqSequenceState::kSteerQuit;
            break;
        }
        case IoniqSequenceState::kSteerQuit: {
            cmd.acc_aps = 0;
            cmd.brk1 = 64;
            if (ioniq_status_.gear_sel_disp != 0)
                cmd.shifter_p = 1;
            else if (ioniq_status_.gear_sel_disp == 0) {
                cmd.shifter_p = 2;
                cmd.enable = 0;
                ioniq_sequence_state_ = IoniqSequenceState::kAccelQuit;
            }
            break;
        }
        case IoniqSequenceState::kAccelQuit: {
            cmd.mode_act = 1;
            ioniq_sequence_state_ = IoniqSequenceState::kEnd;
            break;
        }
        case IoniqSequenceState::kEnd: {
            break;
        }
        Default:
            break;
    }
}

void IoniqCanParser::printData(const IoniqStatus& data) {
    printf("--------------------------------------\n");
    printf("steering_angle: %2.2f\n", data.steering_angle);
    printf("steering_speed: %2.2f\n", data.steering_speed);
    printf("steering_status: %d\n", data.steering_status);
    // printf("steering_tq: %2.2f\n", data.steering_tq);

    printf("accel_pedal_position: %2.2f\n", data.accel_pedal_position);
    printf("longitudinal_accel_speed: %2.2f\n", data.longitudinal_accel_speed);
    printf("lateral_accel_speed: %2.2f\n", data.lateral_accel_speed);
    printf("yaw_rate_sensor: %2.2f\n", data.yaw_rate_sensor);

    printf("brake_active: %2.2f\n", data.brake_active);
    printf("brake_master_cylinder_pressur: %2.2f\n",
           data.brake_master_cylinder_pressur);

    printf("wheel_velocity_fl: %2.2f\n", data.wheel_velocity_fl);
    printf("wheel_velocity_fr: %2.2f\n", data.wheel_velocity_fr);
    printf("wheel_velocity_rl: %2.2f\n", data.wheel_velocity_rl);
    printf("wheel_velocity_rr: %2.2f\n", data.wheel_velocity_rr);

    printf("gear_sel_disp: %2.2f\n", data.gear_sel_disp);

    printf("f_mcu_torque: %2.2f\n", data.f_mcu_torque);
    printf("r_mcu_torque: %2.2f\n", data.r_mcu_torque);

    printf("cluster_odometer: %2.2f\n", data.cluster_odometer);
}

void IoniqCanParser::parseData(const void* nbytes) {
    ioniq_status_.steering_angle = rawToPhysSasAngle(decodeSasAngle(nbytes));
    ioniq_status_.steering_speed = rawToPhysSasSpeed(decodeSasSpeed(nbytes));
    ioniq_status_.steering_status =
        rawToPhysSteeringStatus(decodeSteeringStatus(nbytes));
    ioniq_status_.cluster_odometer =
        rawToPhysClusterOdometer(decodeClusterOdometer(nbytes));
    ioniq_status_.lateral_accel_speed =
        rawToPhysLateralAccelSpeed(decodeLateralAccelSpeed(nbytes));
    ioniq_status_.brake_master_cylinder_pressur =
        rawToPhysBrakeMasterCylinderPressur(
            decodeBrakeMasterCylinderPressur(nbytes));
    ioniq_status_.steering_status =
        rawToPhysSteeringStatus(decodeSteeringStatus(nbytes));
    ioniq_status_.longitudinal_accel_speed =
        rawToPhysLongitudinalAccelSpeed(decodeLongitudinalAccelSpeed(nbytes));
    ioniq_status_.yaw_rate_sensor =
        rawToPhysYawRateSensor(decodeYawRateSensor(nbytes));
    ioniq_status_.wheel_velocity_fr =
        rawToPhysWheelVelocityFr(decodeWheelVelocityFr(nbytes));
    ioniq_status_.brake_active =
        rawToPhysBrakeActive(decodeBrakeActive(nbytes));
    ioniq_status_.wheel_velocity_rl =
        rawToPhysWheelVelocityRl(decodeWheelVelocityRl(nbytes));
    ioniq_status_.wheel_velocity_rr =
        rawToPhysWheelVelocityRr(decodeWheelVelocityRr(nbytes));
    ioniq_status_.wheel_velocity_fl =
        rawToPhysWheelVelocityFl(decodeWheelVelocityFl(nbytes));
    ioniq_status_.steering_torque =
        rawToPhysSteeringTq(decodeSteeringTq(nbytes));
    ioniq_status_.accel_pedal_position =
        rawToPhysAccelPedalPosition(decodeAccelPedalPosition(nbytes));
    ioniq_status_.gear_sel_disp =
        rawToPhysGearSelDisp(decodeGearSelDisp(nbytes));
    ioniq_status_.f_mcu_torque = rawToPhysFMcuTorque(decodeFMcuTorque(nbytes));
    ioniq_status_.r_mcu_torque = rawToPhysRMcuTorque(decodeRMcuTorque(nbytes));
}

vehicle_msgs::IoniqStatus IoniqCanParser::toMsg(const IoniqStatus& in) {
    vehicle_msgs::IoniqStatus msg;
    msg.steering_speed = in.steering_speed;
    msg.steering_angle = in.steering_angle;
    msg.steering_status = in.steering_status;
    msg.cluster_odometer = in.cluster_odometer;
    msg.lateral_accel_speed = in.lateral_accel_speed;
    msg.brake_master_cylinder_pressur = in.brake_master_cylinder_pressur;
    msg.longitudinal_accel_speed = in.longitudinal_accel_speed;
    msg.yaw_rate_sensor = in.yaw_rate_sensor;
    msg.wheel_velocity_fr = in.wheel_velocity_fr;
    msg.brake_active = in.brake_active;
    msg.wheel_velocity_rl = in.wheel_velocity_rl;
    msg.wheel_velocity_rr = in.wheel_velocity_rr;
    msg.wheel_velocity_fl = in.wheel_velocity_fl;
    msg.steering_torque = in.steering_torque;
    msg.accel_pedal_position = in.accel_pedal_position;
    msg.gear_sel_disp = in.gear_sel_disp;
    msg.f_mcu_torque = in.f_mcu_torque;
    msg.r_mcu_torque = in.r_mcu_torque;
    return msg;
}

vehicle_msgs::IoniqControlCommand IoniqCanParser::toMsg(
    const IoniqControlCommnad& in) {
    vehicle_msgs::IoniqControlCommand msg;
    msg.steer = in.steer_str_angl_req;
    msg.accel = in.acc_aps;
    msg.brake = in.brk1;
    return msg;
}

void IoniqCanParser::fromMsg(const vehicle_msgs::IoniqStatus& msg,
                             IoniqStatus& out) {
    out.steering_speed = msg.steering_speed;
    out.steering_angle = msg.steering_angle;
    out.steering_status = msg.steering_status;
    out.cluster_odometer = msg.cluster_odometer;
    out.lateral_accel_speed = msg.lateral_accel_speed;
    out.brake_master_cylinder_pressur = msg.brake_master_cylinder_pressur;
    out.longitudinal_accel_speed = msg.longitudinal_accel_speed;
    out.yaw_rate_sensor = msg.yaw_rate_sensor;
    out.wheel_velocity_fr = msg.wheel_velocity_fr;
    out.brake_active = msg.brake_active;
    out.wheel_velocity_rl = msg.wheel_velocity_rl;
    out.wheel_velocity_rr = msg.wheel_velocity_rr;
    out.wheel_velocity_fl = msg.wheel_velocity_fl;
    out.steering_torque = msg.steering_torque;
    out.accel_pedal_position = msg.accel_pedal_position;
    out.gear_sel_disp = msg.gear_sel_disp;
    out.f_mcu_torque = msg.f_mcu_torque;
    out.r_mcu_torque = msg.r_mcu_torque;
}

void IoniqCanParser::fromMsg(const vehicle_msgs::IoniqControlCommand& msg,
                             IoniqControlCommnad& out) {
    out.steer_str_angl_req = msg.steer;
    out.acc_aps = msg.accel;
    out.brk1 = msg.brake;
}