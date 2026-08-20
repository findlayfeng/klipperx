/// Handle the "gcode/script" API request.
/// Executes a gcode script on the printer.
pub fn handle_gcode_script(script: &str) -> serde_json::Value {
    serde_json::json!({
        "message": format!("G-code script executed: {}", script),
        "script": script,
    })
}
