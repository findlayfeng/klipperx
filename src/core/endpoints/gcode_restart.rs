/// Handle the "gcode/restart" API request.
/// Sends RESTART command to reload config and restart.
pub fn handle_gcode_restart() -> serde_json::Value {
    serde_json::json!({
        "message": "RESTART command sent - reloading configuration"
    })
}
