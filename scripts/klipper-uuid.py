#!/usr/bin/env python3
"""Derive a klipper device uuid from a raw 96-bit stm32 chip id.

Mirrors the firmware's derivation: ``chipid_uuid.h`` takes the low six bytes of
``fasthash64(chip_id, len, CHIPID_UUID_SEED)`` (``lib/fast-hash/fasthash.c``),
and the same six bytes are what a board prints as its canbus/mux uuid.

Usage: klipper-uuid.py 39FFD7054D47323924610951 [...]
"""

import sys

MASK = (1 << 64) - 1
SEED = 0xA16231A7


def mix(h: int) -> int:
    h ^= h >> 23
    h = (h * 0x2127599BF4325C37) & MASK
    h ^= h >> 47
    return h


def fasthash64(buf: bytes, seed: int) -> int:
    m = 0x880355F21E6D1965
    length = len(buf)
    h = seed ^ ((length * m) & MASK)
    whole = length // 8
    for i in range(whole):
        v = int.from_bytes(buf[i * 8 : i * 8 + 8], "little")
        h = ((h ^ mix(v)) * m) & MASK
    v = 0
    for i, byte in enumerate(buf[whole * 8 :]):
        v ^= byte << (8 * i)
    h = ((h ^ mix(v)) * m) & MASK
    return mix(h)


def uuid_for(chip_id_hex: str) -> str:
    chip_id = bytes.fromhex(chip_id_hex)
    digest = fasthash64(chip_id, SEED)
    return digest.to_bytes(8, "little")[:6].hex()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    for arg in sys.argv[1:]:
        print(f"{arg} -> {uuid_for(arg)}")
