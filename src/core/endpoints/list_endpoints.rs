/// Handle the "list_endpoints" API request.
/// Returns all registered endpoint names.
pub fn handle_list_endpoints() -> serde_json::Value {
    let endpoints = vec![
        "info",
        "emergency_stop",
        "register_remote_method",
        "gcode/help",
        "gcode/script",
        "gcode/restart",
        "gcode/firmware_restart",
        "gcode/subscribe_output",
        "objects/list",
        "objects/query",
        "objects/subscribe",
    ];

    serde_json::json!({
        "endpoints": endpoints
    })
}
