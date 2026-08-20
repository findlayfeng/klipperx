/// Handle the "objects/list" API request.
/// Returns all queryable printer objects.
pub fn handle_objects_list() -> serde_json::Value {
    // In a real implementation, this would query the printer object registry
    let objects = vec![
        "print_stats",
        "extruder",
        "heater_bed",
        "fan",
        "gcode_move",
        "toolhead",
        "webhooks",
        "configfile",
        "display_status",
        "idle_timeout",
    ];

    serde_json::json!({
        "objects": objects
    })
}
