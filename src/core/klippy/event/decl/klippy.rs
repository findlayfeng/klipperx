//! `klippy:` events: printer lifecycle.

event!("klippy:mcu_identify");
event!("klippy:connect");
event!("klippy:ready");
event!("klippy:shutdown");
event!("klippy:disconnect");
event!("klippy:firmware_restart");
event!("klippy:notify_mcu_error", { msg: String, details: HashMap<String, Value> });
event!("klippy:analyze_shutdown", { msg: String, details: HashMap<String, Value> });
