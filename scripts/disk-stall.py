#!/usr/bin/env python3
"""disk-stall.py - measure a disk's *stall* profile, not just its average speed.

A card that averages 1200 IOPS can still freeze for a second while its
controller collects garbage. Averages hide that; the p99/max of individual
operations do not. This probe reports both, and it deliberately puts sustained
write pressure on the device while measuring reads, because that is what a
printer does (log writes while replaying a file).

What it measures (each phase is bounded):

    seq_write   a size-mb file in 1 MiB writes + fsync          -> MB/s
    seq_read    the same file after `posix_fadvise(DONTNEED)`   -> MB/s
    rand4k      N random 4 KiB reads, each with fadvise(DONTNEED) so the page
                cache cannot answer it                          -> p50/p99/max us
    stall       a writer thread appending 4 KiB + fdatasync every --interval-ms
                for --stall-seconds, while the main thread keeps doing random
                4 KiB reads                                     -> p99/max ms of
                                                                  both directions

The last phase is the answer to "how bad is it when it stalls": it reports how
many operations took >50 ms / >200 ms / >1000 ms, which is the number that
decides whether log records get dropped or a print replay visibly hitches.

Usage:
    venv/bin/python scripts/disk-stall.py --dir /var/tmp            # full probe
    venv/bin/python scripts/disk-stall.py --dir . --size-mb 16      # smaller file
    venv/bin/python scripts/disk-stall.py --dir . --stall-seconds 0 # skip the stall phase

Output is one JSON object on stdout, so a caller can format it and a human can
still read it with `--pretty`. Nothing outside --dir is touched, and the temp
files are removed at the end.
"""

import argparse
import json
import os
import random
import statistics
import sys
import threading
import time

MIB = 1024 * 1024


def take_latencies(samples_us):
    """(p50, p99, max) in the same units as `samples_us`."""
    if not samples_us:
        return 0.0, 0.0, 0.0
    ordered = sorted(samples_us)
    return (
        statistics.median(ordered),
        ordered[min(len(ordered) - 1, int(len(ordered) * 0.99))],
        ordered[-1],
    )


def drop_cache(fd, offset, length):
    """Forget this range, if the platform allows it (advisory, best effort)."""
    try:
        os.posix_fadvise(fd, offset, length, os.POSIX_FADV_DONTNEED)
    except (AttributeError, OSError):
        pass


def seq_write(path, size_mb):
    buf = bytes(MIB)
    written = 0
    started = time.perf_counter()
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    try:
        for _ in range(size_mb):
            os.write(fd, buf)
            written += MIB
        os.fsync(fd)
    finally:
        os.close(fd)
    elapsed = time.perf_counter() - started
    return written / elapsed / MIB, elapsed


def seq_read(path):
    total = 0
    started = time.perf_counter()
    fd = os.open(path, os.O_RDONLY)
    try:
        size = os.fstat(fd).st_size
        drop_cache(fd, 0, size)
        while True:
            chunk = os.read(fd, MIB)
            if not chunk:
                break
            total += len(chunk)
    finally:
        os.close(fd)
    elapsed = time.perf_counter() - started
    return total / elapsed / MIB, elapsed


def rand4k(path, count, size):
    fd = os.open(path, os.O_RDONLY)
    rng = random.Random(20261021)
    samples = []
    try:
        for _ in range(count):
            offset = rng.randrange(0, max(1, size - 4096)) & ~0xFFF
            drop_cache(fd, offset, 4096)
            started = time.perf_counter()
            data = os.pread(fd, 4096, offset)
            samples.append((time.perf_counter() - started) * 1e6)
            if not data:
                break
    finally:
        os.close(fd)
    return samples


def stall_phase(path, seconds, interval_s, size):
    """Sustained writes + fsyncs, with random reads running against them."""
    write_samples = []
    read_samples = []
    stop = threading.Event()

    def writer():
        fd = os.open(path, os.O_WRONLY)
        offset = 0
        buf = bytes(4096)
        try:
            while not stop.is_set():
                started = time.perf_counter()
                os.pwrite(fd, buf, offset)
                os.fdatasync(fd)
                write_samples.append((time.perf_counter() - started) * 1e3)
                offset = (offset + 4096) % max(4096, size - 4096)
                time.sleep(interval_s)
        finally:
            os.close(fd)

    thread = threading.Thread(target=writer, name="disk-writer", daemon=True)
    thread.start()
    fd = os.open(path, os.O_RDONLY)
    rng = random.Random(7)
    deadline = time.perf_counter() + seconds
    over = {50: 0, 200: 0, 1000: 0}
    try:
        while time.perf_counter() < deadline:
            offset = rng.randrange(0, max(1, size - 4096)) & ~0xFFF
            drop_cache(fd, offset, 4096)
            started = time.perf_counter()
            os.pread(fd, 4096, offset)
            elapsed_ms = (time.perf_counter() - started) * 1e3
            read_samples.append(elapsed_ms)
            for threshold in over:
                if elapsed_ms > threshold:
                    over[threshold] += 1
    finally:
        os.close(fd)
        stop.set()
        thread.join(timeout=10)

    w50 = sum(1 for value in write_samples if value > 50)
    w200 = sum(1 for value in write_samples if value > 200)
    w1000 = sum(1 for value in write_samples if value > 1000)
    return {
        "seconds": seconds,
        "writes": len(write_samples),
        "reads": len(read_samples),
        "write_p50_ms": take_latencies(write_samples)[0],
        "write_p99_ms": take_latencies(write_samples)[1],
        "write_max_ms": take_latencies(write_samples)[2],
        "read_p50_ms": take_latencies(read_samples)[0],
        "read_p99_ms": take_latencies(read_samples)[1],
        "read_max_ms": take_latencies(read_samples)[2],
        "read_over_50ms": over[50],
        "read_over_200ms": over[200],
        "read_over_1000ms": over[1000],
        "write_over_50ms": w50,
        "write_over_200ms": w200,
        "write_over_1000ms": w1000,
    }


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=(
            "Measure a disk's stall profile: sequential throughput, random 4 KiB "
            "latency, and what both do under sustained write pressure. Prints one "
            "JSON object."
        ),
        epilog=(
            "examples:\n"
            "  venv/bin/python scripts/disk-stall.py --dir /var/tmp\n"
            "  venv/bin/python scripts/disk-stall.py --dir . --size-mb 16 --pretty\n"
            "units: MB/s for throughput, microseconds for rand4k, milliseconds for "
            "the stall phase.\n"
            "criteria: rand4k p99 under ~10 ms and stall-phase max under ~200 ms is "
            "healthy for a printer host; a max above 1 s means the controller stalls, "
            "which is what makes log queues overflow (see system_stats.dropped_logs)."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--dir", default=".", help="where to create the temp files")
    parser.add_argument("--size-mb", type=int, default=64, help="file size for the sequential phases")
    parser.add_argument("--random-reads", type=int, default=200, help="4 KiB reads in the rand4k phase")
    parser.add_argument("--stall-seconds", type=float, default=20.0, help="length of the stall phase")
    parser.add_argument("--interval-ms", type=float, default=5.0, help="pause between stall-phase writes")
    parser.add_argument("--pretty", action="store_true", help="indent the JSON")
    args = parser.parse_args(argv)

    if args.size_mb < 4:
        sys.exit("--size-mb must be at least 4")
    os.makedirs(args.dir, exist_ok=True)
    path = os.path.join(args.dir, f".disk-stall-{os.getpid()}.bin")

    result = {"dir": os.path.abspath(args.dir), "file_mb": args.size_mb}
    try:
        write_mbps, write_s = seq_write(path, args.size_mb)
        read_mbps, read_s = seq_read(path)
        size = args.size_mb * MIB
        samples = rand4k(path, args.random_reads, size)
        p50, p99, worst = take_latencies(samples)
        result.update(
            {
                "seq_write_mbps": write_mbps,
                "seq_write_seconds": write_s,
                "seq_read_mbps": read_mbps,
                "seq_read_seconds": read_s,
                "rand4k_count": len(samples),
                "rand4k_p50_us": p50,
                "rand4k_p99_us": p99,
                "rand4k_max_us": worst,
                "rand4k_iops": (1e6 / p50) if p50 else 0.0,
            }
        )
        if args.stall_seconds > 0:
            result["stall"] = stall_phase(
                path, args.stall_seconds, args.interval_ms / 1000.0, size
            )
    finally:
        try:
            os.unlink(path)
        except OSError:
            pass

    json.dump(result, sys.stdout, indent=2 if args.pretty else None)
    sys.stdout.write("\n")


if __name__ == "__main__":
    main()
