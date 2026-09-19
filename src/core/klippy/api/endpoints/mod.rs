//! Endpoint modules — one file per client-facing endpoint.
//!
//! Each endpoint owns its parameters and its response shape, and registers
//! itself with the [`Api`](super::Api) table. The wire contract for every
//! endpoint here is the reference documentation
//! (`docs/klippy/third-party-dev/api-reference.md`); when an endpoint changes,
//! both change together.
//!
//! # Status
//!
//! [`info`](info), [`objects/list`](objects_list), [`objects/query`](objects_query),
//! [`objects/subscribe`](objects_subscribe) and the five [`gcode`](gcode)
//! endpoints are written and registered by [`register`](super::register); the
//! rest of the documented surface is not written yet, so the table below is the
//! checklist.
//!
//! | Endpoint | Status |
//! |---|---|
//! | `info` | done |
//! | `list_endpoints` | done ([`registry`](super::registry)) |
//! | `objects/list`, `objects/query` | done |
//! | `objects/subscribe` | done ([`objects_subscribe`]) |
//! | `gcode/help`, `gcode/script`, `gcode/restart`, `gcode/firmware_restart` | done ([`gcode`]) |
//! | `gcode/subscribe_output` | done ([`gcode`]) |
//! | `emergency_stop` | not started |
//! | `register_remote_method` | not started |
//! | `pause_resume/{pause,resume,cancel}` | not started |
//! | `query_endstops/status` | not started |
//! | `bed_mesh/dump_mesh` | not started |
//! | the `*/dump_*` mux endpoints | not started |

pub mod gcode;
pub mod info;
pub mod objects_list;
pub mod objects_query;
pub mod objects_subscribe;

pub use gcode::{GcodeHelp, GcodeRestart, GcodeScript, GcodeSubscribeOutput};
pub use info::{Info, InfoParams, InfoResponse};
pub use objects_list::ObjectsList;
pub use objects_query::{ObjectsQuery, ObjectsQueryParams};
pub use objects_subscribe::{ObjectsSubscribe, SUBSCRIPTION_REFRESH_TIME};
