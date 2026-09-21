//! `gcode:` events.

event!("gcode:command_error");
event!("gcode:debuginput_exit");
// The payload is the print time at which the restart was requested
// (`klippy/gcode.py:358`). The host has no toolhead yet, so nothing fires it.
event!("gcode:request_restart", { print_time: f64 });
