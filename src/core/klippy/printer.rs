use std::sync::{atomic::{AtomicBool, Ordering}, Arc};
use std::thread::JoinHandle;

use super::config::ConfigSource;

/// A process runner that manages a background loop.
///
/// - `run()` spawns a thread to run the loop body and returns immediately.
/// - On `Drop`, the loop is stopped and the thread is joined.
pub struct ProcessRunner {
    handle: Option<JoinHandle<()>>,
    running: Arc<AtomicBool>,
    source: Option<ConfigSource>,
}

impl ProcessRunner {
    /// Create a new `ProcessRunner` instance with a config source.
    ///
    /// The config is not saved, only the source is tracked.
    pub fn new(source: ConfigSource) -> Self {
        Self {
            handle: None,
            running: Arc::new(AtomicBool::new(false)),
            source: Some(source),
        }
    }

    /// Get the config source.
    pub fn source(&self) -> Option<&ConfigSource> {
        self.source.as_ref()
    }

    /// Spawn a background thread to run the loop. Returns immediately.
    ///
    /// Returns `Ok(())` on success, or an error if spawning the thread fails.
    pub fn run(&mut self) -> Result<(), std::io::Error> {
        if self.running.load(Ordering::SeqCst) {
            return Ok(());
        }

        let running = Arc::clone(&self.running);
        let handle = std::thread::spawn(move || {
            tracing::info!("Loop started");
            while running.load(Ordering::SeqCst) {
                std::thread::sleep(std::time::Duration::from_secs(1));
            }
            tracing::info!("Loop stopped");
        });

        self.handle = Some(handle);
        self.running.store(true, Ordering::SeqCst);

        Ok(())
    }
}

impl Drop for ProcessRunner {
    fn drop(&mut self) {
        if self.running.load(Ordering::SeqCst) {
            tracing::warn!("Stopping loop");
            self.running.store(false, Ordering::SeqCst);
            if let Some(handle) = self.handle.take() {
                let _ = handle.join();
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;

    #[test]
    fn test_process_runner_new() {
        let source = ConfigSource::File(PathBuf::from("/tmp/test.cfg"));
        let runner = ProcessRunner::new(source.clone());
        assert_eq!(runner.source(), Some(&source));
    }

    #[test]
    fn test_process_creation() {
        let source = ConfigSource::None("test".to_string());
        let mut runner = ProcessRunner::new(source);
        let _ = runner.run();
    }
}
