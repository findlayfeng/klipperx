/// Handle the "objects/subscribe" API request.
/// Subscribes to printer object state changes.
pub fn handle_objects_subscribe(
    objects: &serde_json::Value,
    response_template: Option<&str>,
) -> serde_json::Value {
    serde_json::json!({
        "message": "Subscribed to object state changes",
        "objects": objects,
        "response_template": response_template.unwrap_or("none"),
    })
}
