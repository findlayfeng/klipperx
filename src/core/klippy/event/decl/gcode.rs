//! `gcode:` events.

event!("gcode:command_error");
event!("gcode:debuginput_exit");
// The payload is the print time at which the restart was requested
// (`klippy/gcode.py:358`): `GCodeDispatch::request_restart` fires it, and the
// motors (`stepper_enable`) and the fans (`fan`) are handlers that stop
// themselves on it.
event!("gcode:request_restart", { print_time: f64 });
