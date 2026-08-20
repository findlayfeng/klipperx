/// Handle the "gcode/help" API request.
/// Returns help information for gcode commands.
pub fn handle_gcode_help() -> serde_json::Value {
    // In a real implementation, this would query the gcode command table
    serde_json::json!({
        "commands": [
            "G28",
            "G1",
            "M104",
            "M140",
            "M190",
            "M109",
            "RESTART",
            "FIRMWARE_RESTART",
            "HELP",
        ],
        "message": "Use 'gcode/script' with 'HELP <command>' for specific help"
    })
}
