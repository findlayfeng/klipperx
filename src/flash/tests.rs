//! Scenario tests: the flash session against the in-process fake bootloader
//! ([`super::fake`]). Every test body runs under [`bounded`], a watchdog that
//! panics if the body outlives its deadline, so a broken bound in the session
//! fails the suite instead of hanging it.

use std::panic::{catch_unwind, AssertUnwindSafe};
use std::sync::mpsc;
use std::thread;
use std::time::Duration;

use super::fake::{FakeBootloader, Quirk};
use super::frame::command;
use super::session::{flash_with_config, FlashError, SessionConfig};

/// Run `body` on a watchdog thread; fail the test if it exceeds 30 s.
fn bounded(body: impl FnOnce() + Send + 'static) {
    const LIMIT: Duration = Duration::from_secs(30);
    let (tx, rx) = mpsc::channel();
    thread::spawn(move || {
        let _ = tx.send(catch_unwind(AssertUnwindSafe(body)));
    });
    match rx.recv_timeout(LIMIT) {
        Ok(Ok(())) => {}
        Ok(Err(payload)) => std::panic::resume_unwind(payload),
        Err(_) => panic!("test body exceeded its {LIMIT:?} bound"),
    }
}

/// Session timing for tests: same shape as the defaults, millisecond scale.
fn test_config() -> SessionConfig {
    SessionConfig {
        tries: 3,
        read_timeout: Duration::from_millis(40),
        retry_delay: Duration::from_millis(5),
        busy_delay: Duration::from_millis(5),
    }
}

/// 100 bytes → two 64-byte blocks, tail padded with 0xFF.
fn payload() -> Vec<u8> {
    (0..100u8).collect()
}

fn padded_tail() -> Vec<u8> {
    let mut tail = payload()[64..].to_vec();
    tail.resize(64, 0xFF);
    tail
}

#[test]
fn test_flash_runs_the_full_session_and_verifies_the_padded_tail_block() {
    bounded(|| {
        let fake = FakeBootloader::new();
        let mut reader = fake.reader();
        let mut writer = fake.writer();
        let mut lines: Vec<String> = Vec::new();
        let summary =
            flash_with_config(&mut reader, &mut writer, &payload(), &test_config(), |l| {
                lines.push(l.to_string())
            })
            .expect("flash should succeed");
        assert_eq!(summary.start_addr, 0x0800_2000);
        assert_eq!(summary.block_size, 64);
        assert_eq!(summary.block_count, 2);
        assert_eq!(summary.page_count, 2);
        assert_eq!(summary.bytes_flashed, 100);
        assert_eq!(summary.mcu_type, "STM32F103");
        assert_eq!(summary.proto_version, (1, 0, 2));
        assert_eq!(summary.software_version, None);
        // The device holds exactly what was sent, tail block 0xFF-padded —
        // verify read these bytes back and compared them.
        let mem = fake.memory();
        assert_eq!(mem.len(), 2);
        assert_eq!(mem[&0x0800_2000], payload()[..64]);
        assert_eq!(mem[&(0x0800_2000 + 64)], padded_tail());
        // One connect, two blocks, one eof, two verify reads, one complete.
        assert_eq!(fake.seen(command::CONNECT), 1);
        assert_eq!(fake.seen(command::SEND_BLOCK), 2);
        assert_eq!(fake.seen(command::EOF), 1);
        assert_eq!(fake.seen(command::REQUEST_BLOCK), 2);
        assert_eq!(fake.seen(command::COMPLETE), 1);
        assert!(lines.iter().any(|l| l.starts_with("connect:")));
        assert_eq!(
            lines.iter().filter(|l| l.contains("written")).count(),
            2,
            "two block progress lines: {lines:?}"
        );
        assert!(lines.iter().any(|l| l.starts_with("complete:")));
    });
}

#[test]
fn test_a_nacked_block_is_resent_until_the_device_accepts() {
    bounded(|| {
        let fake = FakeBootloader::new();
        // First SEND_BLOCK in the whole run gets a NACK.
        fake.script(command::SEND_BLOCK, 1, Quirk::Nack);
        let mut reader = fake.reader();
        let mut writer = fake.writer();
        let summary =
            flash_with_config(&mut reader, &mut writer, &payload(), &test_config(), |_| {})
                .expect("flash should succeed after the resend");
        assert_eq!(summary.block_count, 2);
        // Block 1: NACK + accepted resend; block 2: first try.
        assert_eq!(fake.seen(command::SEND_BLOCK), 3);
    });
}

#[test]
fn test_busy_is_retried_after_the_busy_delay() {
    bounded(|| {
        let fake = FakeBootloader::new();
        fake.script(command::CONNECT, 1, Quirk::Busy);
        let mut reader = fake.reader();
        let mut writer = fake.writer();
        let summary =
            flash_with_config(&mut reader, &mut writer, &payload(), &test_config(), |_| {})
                .expect("flash should succeed after the busy retry");
        assert_eq!(summary.block_count, 2);
        assert_eq!(fake.seen(command::CONNECT), 2, "busy try + retry");
    });
}

#[test]
fn test_a_response_with_a_bad_crc_is_slid_past_for_the_next_one() {
    bounded(|| {
        let fake = FakeBootloader::new();
        // CONNECT's ack is queued twice: corrupt first, good second.
        fake.script(command::CONNECT, 1, Quirk::CorruptResponse);
        let mut reader = fake.reader();
        let mut writer = fake.writer();
        let summary =
            flash_with_config(&mut reader, &mut writer, &payload(), &test_config(), |_| {})
                .expect("flash should succeed past the corrupt frame");
        assert_eq!(summary.block_count, 2);
        // Both frames arrived inside one try: no resend happened.
        assert_eq!(fake.seen(command::CONNECT), 1);
    });
}

#[test]
fn test_a_response_split_across_two_reads_is_reassembled() {
    bounded(|| {
        let fake = FakeBootloader::new();
        fake.split_responses(true);
        let mut reader = fake.reader();
        let mut writer = fake.writer();
        let summary =
            flash_with_config(&mut reader, &mut writer, &payload(), &test_config(), |_| {})
                .expect("flash should succeed on two-read responses");
        assert_eq!(summary.block_count, 2);
        // 7 responses × 2 chunks × 1 chunk per read.
        assert_eq!(fake.responses_served(), 7);
        assert_eq!(fake.nonempty_reads(), 2 * fake.responses_served());
    });
}

#[test]
fn test_a_device_error_aborts_the_flash() {
    bounded(|| {
        let fake = FakeBootloader::new();
        fake.script(command::CONNECT, 1, Quirk::Error);
        let mut reader = fake.reader();
        let mut writer = fake.writer();
        let err = flash_with_config(&mut reader, &mut writer, &payload(), &test_config(), |_| {})
            .expect_err("0xF2 must abort the session");
        match err {
            FlashError::Device { cmd } => assert_eq!(cmd, command::CONNECT),
            other => panic!("expected Device error, got {other:?}"),
        }
        // Aborted on the first command: no block ever went out.
        assert_eq!(fake.seen(command::SEND_BLOCK), 0);
    });
}

#[test]
fn test_a_silent_device_exhausts_the_tries() {
    bounded(|| {
        let fake = FakeBootloader::new();
        fake.script(command::CONNECT, 1, Quirk::Silent);
        let mut reader = fake.reader();
        let mut writer = fake.writer();
        let err = flash_with_config(&mut reader, &mut writer, &payload(), &test_config(), |_| {})
            .expect_err("silence must exhaust the tries");
        match err {
            FlashError::RetriesExhausted { cmd, tries, .. } => {
                assert_eq!(cmd, command::CONNECT);
                assert_eq!(tries, 3, "one failure per configured try");
            }
            other => panic!("expected RetriesExhausted, got {other:?}"),
        }
        assert_eq!(fake.seen(command::CONNECT), 3, "three unanswered writes");
    });
}

#[test]
fn test_verify_detects_a_block_the_device_stored_corrupted() {
    bounded(|| {
        let fake = FakeBootloader::new();
        // First block is stored flipped, but acknowledged normally.
        fake.script(command::SEND_BLOCK, 1, Quirk::CorruptStored);
        let mut reader = fake.reader();
        let mut writer = fake.writer();
        let err = flash_with_config(&mut reader, &mut writer, &payload(), &test_config(), |_| {})
            .expect_err("verify must catch the flipped block");
        match err {
            FlashError::VerifyMismatch { addr } => assert_eq!(addr, 0x0800_2000),
            other => panic!("expected VerifyMismatch, got {other:?}"),
        }
        assert_eq!(fake.seen(command::REQUEST_BLOCK), 1, "failed on block 1");
        assert_eq!(fake.seen(command::COMPLETE), 0, "never completed");
    });
}
