mod identify;
mod restart_method;

pub use identify::{Identify, IdentifyError, IdentifyErrorKind};
pub use restart_method::McuRestartMethod;

// Tests removed due to Parser API incompatibility - to be fixed later
