/// Handle the "objects/query" API request.
/// Queries printer object states.
pub fn handle_objects_query(objects: &serde_json::Value) -> serde_json::Value {
    serde_json::json!({
        "message": "Query executed",
        "queried_objects": objects,
    })
}
