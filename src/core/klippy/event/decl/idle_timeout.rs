//! `idle_timeout:` events.

// The payload is the print time the transition is dated with
// (`idle_timeout.py:57,96,107`): the toolhead's last move time for `idle`, the
// `check_busy` estimate plus `PIN_MIN_TIME` for `ready`, and — because its
// handler reads the arguments of `toolhead:sync_print_time` in the opposite
// order (`idle_timeout.py:98` against `toolhead.py:267-268`) — the toolhead's
// print time plus `PIN_MIN_TIME` for `printing`.
event!("idle_timeout:idle", { print_time: f64 });
event!("idle_timeout:printing", { print_time: f64 });
event!("idle_timeout:ready", { print_time: f64 });
