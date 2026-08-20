/// Handle the "gcode/subscribe_output" API request.
/// Subscribes to gcode output stream.
pub fn handle_gcode_subscribe_output(response_template: &str) -> serde_json::Value {
    serde_json::json!({
        "message": "Subscribed to gcode output stream",
        "response_template": response_template,
    })
}
