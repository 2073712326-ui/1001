#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>

// Inspect feedback state without adding test-only accessors to the firmware API.
#define private public
#include "M2006.h"
#undef private

namespace {
void check(bool condition, const char* message) {
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", message);
        std::exit(EXIT_FAILURE);
    }
}

bool near(float actual, float expected) {
    return std::fabs(actual - expected) < 0.0001f;
}

void feed(M2006& motor, uint16_t encoder, int16_t rpm = 0,
          int16_t torque = 0, uint8_t reserved6 = 0, uint8_t reserved7 = 0) {
    const uint16_t speed_bits = static_cast<uint16_t>(rpm);
    const uint16_t torque_bits = static_cast<uint16_t>(torque);
    const uint8_t data[8] = {
        static_cast<uint8_t>(encoder >> 8), static_cast<uint8_t>(encoder),
        static_cast<uint8_t>(speed_bits >> 8), static_cast<uint8_t>(speed_bits),
        static_cast<uint8_t>(torque_bits >> 8), static_cast<uint8_t>(torque_bits),
        reserved6, reserved7
    };
    motor.canRxMSgCallback(data);
}

void testTxPacking() {
    const float commands[] = {1000.f, -1000.f, 0.f, 10000.f, -10000.f,
                              50000.f, -50000.f, 1000.9f, -1000.9f};
    const uint16_t expected[] = {0x03E8, 0xFC18, 0, 0x2710, 0xD8F0,
                                 0x2710, 0xD8F0, 0x03E8, 0xFC18};
    for (uint8_t id = 1; id <= 8; ++id) {
        M2006 motor(id, 36.f, POSITION_SPEED);
        const int slot = ((id - 1) % 4) * 2;
        for (unsigned i = 0; i < sizeof(commands) / sizeof(commands[0]); ++i) {
            motor.command_raw_ = commands[i];
            motor.packTxData();
            const uint8_t* data = motor.getTXData();
            check(data[slot] == (expected[i] >> 8), "command high byte and ID slot");
            check(data[slot + 1] == (expected[i] & 0xFF), "command low byte and sign");
            check(std::fabs(motor.command_raw_) <= 10000.f, "current command is limited");
            for (int byte = 0; byte < 8; ++byte) {
                if (byte != slot && byte != slot + 1) {
                    check(data[byte] == 0, "other ESC slots remain zero");
                }
            }
        }
        motor.command_raw_ = std::numeric_limits<float>::quiet_NaN();
        motor.packTxData();
        check(motor.getTXData()[slot] == 0 && motor.getTXData()[slot + 1] == 0,
              "non-finite command becomes zero");
    }
    const uint8_t invalid_ids[] = {0, 9, 255};
    for (uint8_t id : invalid_ids) {
        M2006 motor(id, 36.f, POSITION_SPEED);
        motor.command_raw_ = 1000.f;
        motor.packTxData();
        for (int byte = 0; byte < 8; ++byte) {
            check(motor.getTXData()[byte] == 0, "invalid ID cannot write outside its slot");
        }
    }

    M2006 motor(1, 36.f, POSITION_SPEED);
    motor.spid = PID(1.f, 0.f, 0.f, 0.f, 20000.f, 0.f);
    motor.spid.reset();
    motor.stop_flag = false;
    feed(motor, 0, -1234);
    motor.handle();
    check(motor.getTXData()[0] == 0x04 && motor.getTXData()[1] == 0xD2,
          "handle packs the newly calculated command");
    motor.stop_flag = true;
    motor.handle();
    check(motor.command_raw_ == 0.f, "stop clears calculated current");
    check(motor.getTXData()[0] == 0 && motor.getTXData()[1] == 0,
          "stop overwrites the previous nonzero command");
}
}

int main() {
    M2006 motor(1, 36.f, POSITION_SPEED);
    feed(motor, 4096, 300, -1000);
    check(motor.received_, "first frame marks feedback received");
    check(near(motor.ecd_angle_, 180.f), "encoder 4096 means 180 rotor degrees");
    check(near(motor.angle_, 0.f), "first frame establishes relative zero");
    check(near(motor.speedRpm_, 300.f), "speed is signed 16-bit rotor rpm");
    check(motor.feedback_torque_raw_ == -1000, "negative raw torque is preserved");

    feed(motor, 6144, -300, 1000);
    check(near(motor.angle_, 2.5f), "90 rotor degrees means 2.5 output degrees");
    check(near(motor.speedRpm_, -300.f), "negative rpm is preserved");
    check(motor.feedback_torque_raw_ == 1000, "positive raw torque is preserved");
    feed(motor, 4096);
    check(near(motor.angle_, 0.f), "reverse movement subtracts from position");
    feed(motor, 4096);
    check(near(motor.delta_angle_, 0.f), "stationary frame has zero increment");

    M2006 forward(1, 36.f, POSITION_SPEED);
    feed(forward, 8191);
    feed(forward, 0);
    const float one_count = 360.f / 8192.f / 36.f;
    check(near(forward.delta_angle_, one_count), "8191 -> 0 is one positive count");
    check(near(forward.angle_, one_count), "forward wrap does not jump");
    feed(forward, 1);
    check(near(forward.angle_, 2.f * one_count), "previous angle updates after wrap");

    M2006 reverse(1, 36.f, POSITION_SPEED);
    feed(reverse, 0);
    feed(reverse, 8191);
    check(near(reverse.delta_angle_, -one_count), "0 -> 8191 is one negative count");
    check(near(reverse.angle_, -one_count), "reverse wrap does not jump");
    feed(reverse, 8190);
    check(near(reverse.angle_, -2.f * one_count), "reverse continues after wrap");

    M2006 turns(1, 36.f, POSITION_SPEED);
    feed(turns, 0);
    for (int i = 1; i <= 16; ++i) {
        feed(turns, static_cast<uint16_t>((i % 4) * 2048));
    }
    check(near(turns.angle_, 40.f), "four rotor turns accumulate to 40 output degrees");
    for (int i = 15; i >= 0; --i) {
        feed(turns, static_cast<uint16_t>((i % 4) * 2048));
    }
    check(near(turns.angle_, 0.f), "four reverse turns return to relative zero");

    feed(motor, 4096, INT16_MIN, INT16_MAX, 0xAB, 0xCD);
    check(near(motor.speedRpm_, -32768.f), "minimum signed rpm is preserved");
    check(motor.feedback_torque_raw_ == 32767, "maximum raw torque is preserved");
    check(near(motor.angle_, 0.f), "reserved bytes do not affect position");
    feed(motor, 4096, INT16_MAX, INT16_MIN, 0xFF, 0xFF);
    check(near(motor.speedRpm_, 32767.f), "maximum signed rpm is preserved");
    check(motor.feedback_torque_raw_ == -32768, "minimum raw torque is preserved");

    testTxPacking();
    std::puts("PASS: M2006 feedback decoding and C610 current command packing");
}
