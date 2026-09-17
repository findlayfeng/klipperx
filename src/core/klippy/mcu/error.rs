/// Error returned by [`Mcu::call`](super::Mcu::call).
#[derive(Debug)]
pub enum McuCallError {
    /// The command name is not registered in the message parser.
    CommandNotFound(String),
    /// The command already has a callback bound — `call` is only for
    /// synchronous request/response pairs.
    CommandHasCallback(String),
    /// The send buffer is full and the command could not be queued.
    SendFailed(String),
    /// The expected response did not arrive before `timeout` elapsed.
    Timeout(String),
}

impl std::fmt::Display for McuCallError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            McuCallError::CommandNotFound(name) => {
                write!(f, "command not found: {}", name)
            }
            McuCallError::CommandHasCallback(name) => {
                write!(f, "command already has callback: {}", name)
            }
            McuCallError::SendFailed(msg) => write!(f, "send failed: {}", msg),
            McuCallError::Timeout(msg) => write!(f, "timeout: {}", msg),
        }
    }
}

impl std::error::Error for McuCallError {}
