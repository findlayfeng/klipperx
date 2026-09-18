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
//! Only `info` exists so far, and only its definition: the handler body is a
//! `todo!()`, so the endpoint is registered and dispatchable but panics when
//! called. The rest of the documented surface is not written yet — the table
//! below is the checklist.
//!
//! | Endpoint | Status |
//! |---|---|
//! | `info` | defined, handler `todo!()` |
//! | `list_endpoints` | done ([`registry`](super::registry)) |
//! | `emergency_stop` | not started |
//! | `register_remote_method` | not started |
//! | `objects/list`, `objects/query`, `objects/subscribe` | not started |
//! | `gcode/help`, `gcode/script`, `gcode/restart` | not started |
//! | `gcode/firmware_restart`, `gcode/subscribe_output` | not started |
//! | `pause_resume/{pause,resume,cancel}` | not started |
//! | `query_endstops/status` | not started |
//! | `bed_mesh/dump_mesh` | not started |
//! | the `*/dump_*` mux endpoints | not started |

pub mod info;

pub use info::{Info, InfoParams, InfoResponse};
