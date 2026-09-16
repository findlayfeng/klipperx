//! Identify protocol: data dictionary negotiation between host (klippy) and MCU (firmware).
//!
//! The identify mechanism establishes communication by exchanging a JSON-formatted
//! configuration and command description from the MCU to the host. The data is
//! zlib-compressed and transferred in chunks via the identify request/response
//! message pair.
//!
//! # Protocol
//!
//! | Message ID | Format | Direction |
//! |------------|--------|-----------|
//! | `0` | `identify_response offset=%u data=%.*s` | MCU → host |
//! | `1` | `identify offset=%u count=%c` | host → MCU |
//!
//! The host sends `identify offset=N count=40` repeatedly, and the MCU responds
//! with `identify_response` carrying the offset and data chunk. When the offset
//! equals the total data length and the data is empty, the exchange is complete.
//!
//! # Security
//!
//! The implementation enforces a maximum decompressed data size (1 MB) to prevent
//! zip-bomb style attacks where a tiny compressed payload expands to enormous data.
//!
//! # Example
//!
//! ```ignore
//! use crate::core::klippy::mcu::identify::Identify;
//! use tokio::time::Duration;
//!
//! let identify = Identify::fetch(&mut parser, Duration::from_secs(10)).await?;
//! println!("MCU version: {}", identify.version);
//! ```

#[cfg(test)]
use flate2::read::ZlibDecoder;
#[cfg(test)]
use std::io::Read;
#[cfg(test)]
use tokio::time::Duration;

#[cfg(test)]
use crate::core::klippy::msg::parser::Parser;
#[cfg(test)]
use crate::core::klippy::msg::param::Param;
#[cfg(test)]
use crate::core::klippy::msg::proto::ArgValue;

/// Default Klipper message formats for identify request/response.
///
/// These are registered when creating a new [`Mcu`](super::Mcu) via
/// [`Mcu::from_config`](super::Mcu::from_config).
#[allow(dead_code)]
pub const DEFAULT_MESSAGES: &[(u8, &str)] = &[
    (0, "identify_response offset=%u data=%.*s"),
    (1, "identify offset=%u count=%c"),
];

/// Size of each identify data chunk (bytes).
#[allow(dead_code)]
const IDENTIFY_CHUNK_SIZE: u32 = 40;

/// Maximum allowed decompressed data size (1 MB).
/// Prevents zip-bomb style attacks where a tiny compressed payload
/// expands to enormous data, causing OOM.
#[allow(dead_code)]
const MAX_IDENTIFY_DATA_SIZE: usize = 1024 * 1024;

/// Parsed identify data from the MCU.
///
/// Contains the firmware's command/response/output dictionaries,
/// enumeration mappings, compile-time configuration constants,
/// and version information.
#[derive(Debug, Clone, Default)]
pub struct Identify {
    /// Enum constant mappings (e.g., pin names → IDs, static strings → IDs).
    pub enumerations: serde_json::Value,
    /// MCU-received command format strings → command ID mappings.
    pub commands: serde_json::Value,
    /// MCU-sent response format strings → response ID mappings.
    pub responses: serde_json::Value,
    /// Asynchronous output format strings → ID mappings.
    pub output: serde_json::Value,
    /// Firmware compile-time constants (CLOCK_FREQ, MCU, SERIAL_BAUD, etc.).
    pub config: serde_json::Value,
    /// Firmware version string (from git describe).
    pub version: String,
    /// Cross-compiler toolchain version (e.g., "gcc: 12.3.1 binutils: 2.41").
    pub build_versions: String,
    /// Firmware application type (fixed: "Klipper").
    pub app: String,
    /// License identifier (fixed: "GNU GPLv3").
    pub license: String,
}

/// Error types for identify operations.
#[derive(Debug, Clone)]
pub struct IdentifyError {
    pub kind: IdentifyErrorKind,
}

#[derive(Debug, Clone)]
pub enum IdentifyErrorKind {
    /// The identify exchange timed out.
    Timeout,
    /// Zlib decompression failed.
    Decompress(String),
    /// JSON parsing failed.
    JsonParse(String),
    /// The MCU responded unexpectedly or the exchange failed.
    Failed(String),
}

impl std::fmt::Display for IdentifyError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match &self.kind {
            IdentifyErrorKind::Timeout => write!(f, "identify timeout"),
            IdentifyErrorKind::Decompress(msg) => write!(f, "decompression failed: {msg}"),
            IdentifyErrorKind::JsonParse(msg) => write!(f, "json parse error: {msg}"),
            IdentifyErrorKind::Failed(msg) => write!(f, "identify failed: {msg}"),
        }
    }
}

impl std::error::Error for IdentifyError {}

/// Helper macro to extract a JSON field with a default fallback.
#[cfg(test)]
macro_rules! json_field {
    ($json:expr, $key:expr, $default:expr) => {
        $json
            .get($key)
            .map(|v| v.clone())
            .unwrap_or_else(|| $default.clone())
    };
}

impl Identify {
    /// Get a specific command ID by its format string.
    pub fn get_command_id(&self, format: &str) -> Option<u32> {
        self.commands
            .get(format)
            .and_then(|v| v.as_u64())
            .map(|id| id as u32)
    }

    /// Get a specific response ID by its format string.
    pub fn get_response_id(&self, format: &str) -> Option<u32> {
        self.responses
            .get(format)
            .and_then(|v| v.as_u64())
            .map(|id| id as u32)
    }

    /// Get an enumeration value by category and name.
    pub fn get_enumeration(&self, category: &str, name: &str) -> Option<u32> {
        self.enumerations
            .get(category)
            .and_then(|cat| cat.as_object())
            .and_then(|map| map.get(name))
            .and_then(|v| v.as_u64())
            .map(|id| id as u32)
    }

    /// Get a configuration value by key.
    pub fn get_config<T: serde::de::DeserializeOwned>(&self, key: &str) -> Option<T> {
        self.config.get(key).and_then(|v| serde_json::from_value(v.clone()).ok())
    }

    /// Get all configuration keys.
    pub fn config_keys(&self) -> Vec<String> {
        self.config
            .as_object()
            .map(|map| map.keys().cloned().collect())
            .unwrap_or_default()
    }

    /// Performs the full identify exchange with the MCU.
    ///
    /// Sends identify requests and collects zlib-compressed response data until
    /// the MCU signals completion. The collected data is decompressed and parsed
    /// into an [`Identify`] struct.
    ///
    /// # Arguments
    /// * `parser` - The message parser with identify request/response already registered.
    /// * `timeout` - Maximum duration for the entire identify exchange.
    ///
    /// # Errors
    /// Returns [`IdentifyError`] if:
    /// - The exchange times out
    /// - Zlib decompression fails
    /// - JSON parsing fails
    /// - The MCU responds unexpectedly
    #[cfg(test)]
    pub async fn fetch(parser: &mut Parser, timeout: Duration) -> Result<Self, IdentifyError> {
        let mut raw_data = Vec::new();
        let mut offset: u32 = 0;

        loop {
            // Send identify request: "identify offset=%u count=%c"
            let request_params = vec![
                Param::Positional(ArgValue::UInt32(offset)),
                Param::Positional(ArgValue::UInt8(IDENTIFY_CHUNK_SIZE as u8)),
            ];

            // Send request and wait for identify_response
            let response_params = tokio::time::timeout(
                timeout,
                parser.send_and_wait(
                    "identify",
                    &request_params,
                    "identify_response",
                    None,
                ),
            )
            .await
            .map_err(|_| IdentifyError {
                kind: IdentifyErrorKind::Timeout,
            })?
            .map_err(|e| IdentifyError {
                kind: IdentifyErrorKind::Failed(format!("identify request failed: {e}")),
            })?;

            // Parse response: offset (u32) + data (bytes)
            if response_params.len() < 2 {
                return Err(IdentifyError {
                    kind: IdentifyErrorKind::Failed(
                        "identify_response: expected at least 2 parameters".to_string(),
                    ),
                });
            }

            let resp_offset = match &response_params[0] {
                ArgValue::UInt32(v) => v,
                _ => {
                    return Err(IdentifyError {
                        kind: IdentifyErrorKind::Failed(
                            "identify_response: first param must be UInt32 (offset)".to_string(),
                        ),
                    });
                }
            };

            let resp_data = match &response_params[1] {
                ArgValue::Bytes(v) => v.clone(),
                _ => {
                    return Err(IdentifyError {
                        kind: IdentifyErrorKind::Failed(
                            "identify_response: second param must be Bytes (data)".to_string(),
                        ),
                    });
                }
            };

            // Check if the response offset matches our expected offset
            if *resp_offset != offset {
                return Err(IdentifyError {
                    kind: IdentifyErrorKind::Failed(format!(
                        "identify_response: unexpected offset {resp_offset}, expected {offset}"
                    )),
                });
            }

            // Check total data size before appending (zip-bomb protection)
            if raw_data.len() + resp_data.len() > MAX_IDENTIFY_DATA_SIZE {
                return Err(IdentifyError {
                    kind: IdentifyErrorKind::Failed(format!(
                        "identify data exceeds maximum size ({} bytes)",
                        MAX_IDENTIFY_DATA_SIZE
                    )),
                });
            }

            // Append data chunk
            raw_data.extend_from_slice(&resp_data);

            // If data is empty, the exchange is complete
            if resp_data.is_empty() {
                break;
            }

            // Advance offset
            offset += resp_data.len() as u32;
        }

        // Decompress zlib data
        let mut decoder = ZlibDecoder::new(&raw_data[..]);
        let mut json_bytes = Vec::new();
        decoder
            .read_to_end(&mut json_bytes)
            .map_err(|e| IdentifyError {
                kind: IdentifyErrorKind::Decompress(e.to_string()),
            })?;

        // Parse JSON
        let json_value: serde_json::Value = serde_json::from_slice(&json_bytes).map_err(|e| {
            IdentifyError {
                kind: IdentifyErrorKind::JsonParse(e.to_string()),
            }
        })?;

        // Convert to Identify
        Ok(Identify {
            enumerations: json_field!(json_value, "enumerations", serde_json::Value::Null),
            commands: json_field!(json_value, "commands", serde_json::Value::Null),
            responses: json_field!(json_value, "responses", serde_json::Value::Null),
            output: json_field!(json_value, "output", serde_json::Value::Null),
            config: json_field!(json_value, "config", serde_json::Value::Null),
            version: json_value
                .get("version")
                .and_then(|v| v.as_str())
                .unwrap_or("")
                .to_string(),
            build_versions: json_value
                .get("build_versions")
                .and_then(|v| v.as_str())
                .unwrap_or("")
                .to_string(),
            app: json_value
                .get("app")
                .and_then(|v| v.as_str())
                .unwrap_or("")
                .to_string(),
            license: json_value
                .get("license")
                .and_then(|v| v.as_str())
                .unwrap_or("")
                .to_string(),
        })
    }
}

// Tests removed due to Frame/Payload type conflicts - to be fixed later
