//! `toolhead:` events.

event!("toolhead:manual_move");
event!("toolhead:set_position");
// The print time the toolhead raised and the numbers it was raised from, in
// upstream's send order (`_calc_print_time`, `klippy/toolhead.py:261-268`):
// `curtime`, the MCU's estimate at that instant, and the new print time. The
// only consumer, `idle_timeout`, names the last two the other way round
// (`idle_timeout.py:98`) — see `event/decl/idle_timeout.rs`.
event!("toolhead:sync_print_time", { curtime: f64, est_print_time: f64, print_time: f64 });
event!("toolhead:update_extra_axes");
