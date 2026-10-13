# M2006 feedback regression test

Run from the repository root with a host C++ compiler (no STM32 hardware required):

```powershell
New-Item -ItemType Directory -Force build/tests | Out-Null
g++ -std=c++11 -Wall -Wextra -IUserCode tests/test_m2006_feedback.cpp UserCode/M2006.cpp UserCode/pid.cpp -o build/tests/test_m2006_feedback.exe
./build/tests/test_m2006_feedback.exe
```

The test covers C610 big-endian fields, signed 16-bit speed and raw torque,
first-frame initialization, stationary feedback, forward/reverse encoder wrap,
multiple turns, the reduction ratio, and reserved bytes. Position is relative
to the first feedback frame; speed remains rotor rpm. Torque is kept raw because
the supplied C610 manual does not specify its conversion to N.m.

Angle unwrapping assumes rotor motion between consecutive feedback frames is
less than 180 degrees. Long feedback gaps can make turn counting ambiguous.

Transmit tests also cover positive/negative commands, the +/-10000 current limit,
fractional truncation, ID slots 1..8, invalid IDs, non-finite input, and clearing a
previous nonzero command when stopped. `handle()` updates the transmit buffer;
call it before retrieving and sending the frame. The caller must select CAN ID
0x200 for ESC IDs 1..4 or 0x1FF for ESC IDs 5..8. Separate instances have separate
buffers; a shared multi-motor frame must combine their slots before transmission.
