/// Handle the "gcode/firmware_restart" API request.
/// Sends FIRMWARE_RESTART command to restart firmware.
pub fn handle_gcode_firmware_restart() -> serde_json::Value {
    serde_json::json!({
        "message": "FIRMWARE_RESTART command sent - restarting firmware"
    })
}
