use std::env;

/// Handle the "info" API request.
/// Returns printer state, hostname, klipper path, python path,
/// process ID, user/group ID, and software version.
pub fn handle_info(client_info: Option<&str>) -> serde_json::Value {
    let hostname = env::var("HOSTNAME").unwrap_or_default();
    let python_path = env::var("PATH").unwrap_or_default();
    let process_id = std::process::id();

    // In a real implementation, these would come from klippy start_args
    let software_version = env::var("KLIPPER_VERSION").unwrap_or_else(|_| "0.1.0".to_string());
    let klipper_path = env::var("KLIPPER_PATH").unwrap_or_else(|_| "/usr/share/klipper".to_string());
    let cpu_info = env::var("KLIPPER_CPU_INFO").unwrap_or_else(|_| "x86_64".to_string());
    let log_file = env::var("KLIPPER_LOG_FILE").ok();
    let config_file = env::var("KLIPPER_CONFIG_FILE").ok();

    // Safe wrappers for getuid/getgid
    let user_id = unsafe { libc::getuid() };
    let group_id = unsafe { libc::getgid() };

    let mut response = serde_json::json!({
        "state": "ready",
        "state_message": "Printer is ready",
        "hostname": hostname,
        "klipper_path": klipper_path,
        "python_path": python_path,
        "process_id": process_id,
        "user_id": user_id,
        "group_id": group_id,
        "software_version": software_version,
        "cpu_info": cpu_info,
    });

    if let Some(log_file) = log_file {
        response["log_file"] = serde_json::json!(log_file);
    }
    if let Some(config_file) = config_file {
        response["config_file"] = serde_json::json!(config_file);
    }
    if let Some(client_info) = client_info {
        response["client_info"] = serde_json::json!(client_info);
    }

    response
}
