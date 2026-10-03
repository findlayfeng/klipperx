//! Endpoint modules — one file per client-facing endpoint.
//!
//! Each endpoint owns its parameters and its response shape, and installs
//! itself with the [`Api`](super::Api) table. The wire contract for every
//! endpoint here is the reference documentation
//! (`docs/klippy/third-party-dev/api-reference.md`); when an endpoint changes,
//! both change together.
//!
//! A module with endpoints declares an `install` function with the
//! `EndpointInstaller` signature and marks it with an
//! `endpoint!` declaration; `build.rs` collects those into the table
//! [`register`](super::register) walks, so adding an endpoint group does not edit
//! a central list.
//!
//! # Status
//!
//! [`info`](info), [`objects/list`](objects_list), [`objects/query`](objects_query),
//! [`objects/subscribe`](objects_subscribe), the five [`gcode`](gcode)
//! endpoints, [`emergency_stop`](emergency_stop),
//! [`register_remote_method`](register_remote_method) and the three
//! [`pause_resume`](pause_resume) endpoints are written and registered by
//! [`register`](super::register); the rest of the documented surface is not
//! written yet, so the table below is the checklist.
//!
//! | Endpoint | Status |
//! |---|---|
//! | `info` | done |
//! | `list_endpoints` | done ([`registry`](super::registry)) |
//! | `objects/list`, `objects/query` | done |
//! | `objects/subscribe` | done ([`objects_subscribe`]) |
//! | `gcode/help`, `gcode/script`, `gcode/restart`, `gcode/firmware_restart` | done ([`gcode`]) |
//! | `gcode/subscribe_output` | done ([`gcode`]) |
//! | `emergency_stop` | done ([`emergency_stop`]) |
//! | `query_endstops/status` | done ([`query_endstops`]) |
//! | `register_remote_method` | done ([`register_remote_method`]) |
//! | `pause_resume/{pause,resume,cancel}` | done ([`pause_resume`]) |
//! | `bed_mesh/dump_mesh` | not started |
//! | the `*/dump_*` mux endpoints | mechanism ready ([`WebhooksStatus::register_mux_endpoint`](super::webhooks::WebhooksStatus::register_mux_endpoint); instances are re-registered per config load and detached on the way out); `ldc1612` / `adxl345` / `mpu9250` / `load_cell` consumed; the rest arrive with their extras |

/// Declare that this module installs its endpoints. Expands to nothing;
/// `build.rs` scans it and lists the named function in the generated table.
///
/// The name is usually `install`; it is resolved as a sibling of the declaring
/// module (a path may also be given), and it must have the
/// `EndpointInstaller` signature.
macro_rules! endpoint {
    ($($tokens:tt)*) => {};
}

pub mod emergency_stop;
pub mod gcode;
pub mod info;
pub mod objects_list;
pub mod objects_query;
pub mod objects_subscribe;
pub mod pause_resume;
pub mod query_endstops;
pub mod register_remote_method;

pub use emergency_stop::EmergencyStop;
pub use gcode::{GcodeHelp, GcodeRestart, GcodeScript, GcodeSubscribeOutput};
pub use info::{Info, InfoParams, InfoResponse};
pub use objects_list::ObjectsList;
pub use objects_query::{ObjectsQuery, ObjectsQueryParams};
pub use objects_subscribe::{ObjectsSubscribe, SUBSCRIPTION_REFRESH_TIME};
pub use pause_resume::PauseResumeEndpoint;
