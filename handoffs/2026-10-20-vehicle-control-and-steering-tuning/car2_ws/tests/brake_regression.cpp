#include <cassert>
#include "../firmware/teensys/brake.cpp"

static void feedback(int value) { mock_analog_read = value; feedback_raw = value; }

int main() {
    setup();
    stale = false; cmd_estop = false;
    feedback(cfg::ADC_RELEASED); cmd_brake = 1.0f; controlStep();
    assert(mock_pwm[18] > 0 && mock_pwm[19] == 0 && motion == MOTION_EXTEND);
    feedback(cfg::ADC_APPLIED); cmd_brake = 0.0f; controlStep();
    assert(mock_pwm[19] > 0 && mock_pwm[18] == 0 && motion == MOTION_RETRACT);
    feedback(cfg::ADC_APPLIED); cmd_brake = 1.0f; controlStep();
    assert(mock_pwm[19] == 0 && mock_pwm[18] == 0 && motion == MOTION_STOP);
    feedback(cfg::ADC_RELEASED); cmd_brake = 0.0f; cmd_estop = true; controlStep();
    assert(mock_pwm[18] > 0 && mock_pwm[19] == 0);
    cmd_estop = false; stale = true; controlStep();
    assert(mock_pwm[18] > 0 && mock_pwm[19] == 0);
    // Measured feedback rises on application. Confirm speed, target stopping,
    // and overshoot correction against physical readings, not old calibration.
    stale = false; cmd_estop = false; cmd_brake = 1.0f;
    feedback(130); controlStep();
    assert(mock_pwm[18] == 255 && mock_pwm[19] == 0);
    feedback(267); controlStep();
    assert(mock_pwm[18] == 255 && mock_pwm[19] == 0);
    feedback(277); controlStep();
    assert(mock_pwm[18] == 0 && mock_pwm[19] == 0);
    feedback(300); controlStep();
    assert(mock_pwm[19] > 0 && mock_pwm[18] == 0);
    cmd_brake = 0.5f; feedback(175); controlStep();
    assert(mock_pwm[18] == 0 && mock_pwm[19] == 0);
    feedback(74); sendStatus();
    assert((can.outgoing.back().buf[5] & 3) == 1);
    assert(can.outgoing.back().buf[2] == 0);
    feedback(277); sendStatus();
    assert((can.outgoing.back().buf[5] & 3) == 2);
    assert(can.outgoing.back().buf[2] == 83); // nominal 0.83in of full stroke
    feedback(175); sendStatus();
    assert((can.outgoing.back().buf[5] & 3) == 0);
    feedback(564); sendStatus();
    assert(can.outgoing.back().buf[2] == 200);
    // Exercise normal protected CAN control, including held Space-style requests.
    rx = {};
    uint8_t sequence = 0;
    CAN_message_t command;
    uint8_t payload[2] = {0, 255};
    feedback(130);
    for (int i = 0; i < 100; ++i) {
        mock_now += 20;
        dbw_pack_cmd(command, DBW_ID_BRAKE_CMD, sequence, payload, 2);
        can.incoming.push_back(command); loop();
        assert(mock_pwm[18] == 255 && mock_pwm[19] == 0 && !stale && !cmd_estop);
    }
    feedback(277); mock_now += 5; loop();
    assert(mock_pwm[18] == 0 && mock_pwm[19] == 0);
    payload[1] = 0;
    dbw_pack_cmd(command, DBW_ID_BRAKE_CMD, sequence, payload, 2);
    can.incoming.push_back(command); mock_now += 5; loop();
    assert(mock_pwm[19] == 255 && mock_pwm[18] == 0);
    feedback(74); mock_now += 5; loop();
    assert(mock_pwm[18] == 0 && mock_pwm[19] == 0);
    mock_now += 101; loop();
    assert(stale && mock_pwm[18] == 255 && mock_pwm[19] == 0);
    puts("PASS brake: apply pin18, release pin19, target stop, E-stop and stale apply");
}
