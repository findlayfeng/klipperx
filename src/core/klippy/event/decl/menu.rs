//! `menu:` events. Upstream sends these as `"menu:" + event`, so the four names
//! are declared here explicitly.

event!("menu:populate");
event!("menu:init");
event!("menu:begin");
event!("menu:exit");
