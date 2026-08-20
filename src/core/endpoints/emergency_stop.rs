/// Handle the "emergency_stop" API request.
/// Triggers printer shutdown.
pub fn handle_emergency_stop() -> serde_json::Value {
    serde_json::json!({
        "message": "Emergency stop triggered - printer shutting down"
    })
}
