//! Temporary scratch test for the TMC-UART unit: load a few real printer
//! configs the way `upstream.rs::run_phases` does and report each one's
//! outcome. Not committed.

use std::path::PathBuf;
use std::sync::Arc;

use crate::core::klippy::config::value::ConfigValue;
use crate::core::klippy::config::Config;
use crate::core::klippy::printer::{Printer, PrinterState};
use crate::core::klippy::reactor::TokioReactor;

fn config_dir() -> PathBuf {
    klipperx_test_support::klipper_dir().join("config")
}

fn dict(name: &str) -> PathBuf {
    klipperx_test_support::test_dicts_dir().join(name)
}

fn injected(text: &str, dictionary: &std::path::Path) -> Config {
    let (config, _) = Config::from_text(text).expect("parses");
    let mut out = Config::new();
    for section in config.sections_vec() {
        let mut section = section.clone();
        if section.id == "mcu" {
            for key in [
                "serial",
                "baud",
                "canbus_uuid",
                "canbus_interface",
                "host_library",
                "test",
            ] {
                section.parameters.remove(key);
            }
            section.parameters.insert(
                "test".to_string(),
                ConfigValue::Single(format!("dict={}", dictionary.display())),
            );
        }
        out.add_section(section);
    }
    out
}

async fn load_case(file: &str, dictionary: &str) -> Result<(), String> {
    let path = config_dir().join(file);
    let text = std::fs::read_to_string(&path).map_err(|e| format!("{}: {e}", path.display()))?;
    let config = injected(&text, &dict(dictionary));
    let reactor = Arc::new(TokioReactor::new(tokio::runtime::Handle::current()));
    let printer = Arc::new(Printer::new(reactor));
    let mut start_args = crate::core::klippy::api::StartArgs::collect(file, None);
    start_args.debug_output = Some("_test_output".to_string());
    printer.set_start_args(Arc::new(start_args));
    let setup = async {
        printer.load_config(&config).map_err(|e| e.to_string())?;
        if tokio::time::timeout(std::time::Duration::from_secs(10), printer.bring_up())
            .await
            .is_err()
        {
            return Err("bring_up timed out".to_string());
        }
        let state = printer.get_state_message();
        if state.category != PrinterState::Ready {
            return Err(format!("not ready: {}", state.message));
        }
        Ok(())
    }
    .await;
    printer.teardown();
    setup
}

#[tokio::test(flavor = "multi_thread")]
async fn scratch_load_representative_configs() {
    let cases = [
        ("generic-bigtreetech-skr-mini-e3-v2.0.cfg", "stm32f103.dict"),
        ("generic-fysetc-cheetah-v2.0.cfg", "stm32f401.dict"),
        ("printer-prusa-mini-plus-2020.cfg", "stm32f407.dict"),
        ("generic-th3d-ezboard-lite-v1.2.cfg", "lpc176x.dict"),
        ("printer-biqu-b1-se-plus-2022.cfg", "stm32f407.dict"),
    ];
    for (file, dictionary) in cases {
        match load_case(file, dictionary).await {
            Ok(()) => println!("SCRATCH {file}: Ok"),
            Err(err) => println!("SCRATCH {file}: FAIL {err}"),
        }
    }
}
