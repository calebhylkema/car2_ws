#include <cassert>
#include "../firmware/teensys/brake_bench_test.cpp"
static void send(char c) { Serial.input.push_back(c); loop(); }
static void stopped() { assert(mock_pwm[18] == 0 && mock_pwm[19] == 0); }
int main() {
    mock_analog_read = 300; setup(); stopped();
    send('a'); assert(mock_pwm[18] == 255 && mock_pwm[19] == 0);
    for (int i = 0; i < 100; ++i) {
        mock_now += 20; send('a'); assert(mock_pwm[18] == 255 && mock_pwm[19] == 0);
    }
    send('x'); stopped();
    send('r'); assert(mock_pwm[19] == 255 && mock_pwm[18] == 0);
    mock_now += 151; loop(); stopped();
    send('a'); mock_analog_read = 564; loop(); stopped();
    mock_analog_read = 560; send('a'); stopped();
    send('r'); assert(mock_pwm[19] == 255 && mock_pwm[18] == 0);
    mock_analog_read = 74; loop(); stopped();
    mock_analog_read = 80; send('r'); stopped();
    send('a'); assert(mock_pwm[18] == 255 && mock_pwm[19] == 0);
    Serial.connected = false; loop(); stopped();
    Serial.connected = true; mock_analog_read = 1023; send('a'); stopped();
    mock_analog_read = 0; send('r'); stopped();
    mock_analog_read = 300; mock_now = 0xfffffff0u; send('a');
    mock_now += 100; loop(); assert(mock_pwm[18] == 255);
    mock_now += 51; loop(); stopped();
    puts("PASS brake hold: continuous drive, release stop, timeout, limits, latch, USB loss, rails, clock wrap");
}
