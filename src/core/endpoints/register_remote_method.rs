/// Handle the "register_remote_method" API request.
/// Registers a remote method for RPC communication.
pub fn handle_register_remote_method(
    response_template: &str,
    remote_method: &str,
) -> serde_json::Value {
    serde_json::json!({
        "message": format!(
            "Remote method '{}' registered for RPC",
            remote_method
        ),
        "remote_method": remote_method,
        "response_template": response_template,
    })
}
