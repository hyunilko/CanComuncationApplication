# mmwave_profile_cfg.py
# Binary (packed) serialization for MMWave_ProfileComCfg
# Wire format: little-endian, packed (no padding)
# Layout (bytes): <BBB H f BB  => total 11 bytes
#
#   digOutputSampRate     : uint8
#   digOutputBitsSel      : uint8
#   dfeFirSel             : uint8
#   numOfAdcSamples       : uint16 (LE)
#   chirpRampEndTimeus    : float  (IEEE-754, LE)
#   chirpRxHpfSel         : uint8
#   chirpTxMimoPatSel     : uint8
#
# IMPORTANT: On the firmware side, use a packed struct (pragma pack(1) or __attribute__((packed)))
# and little-endian interpretation to match this wire format. See README_PATCH.txt for a C snippet.

from __future__ import annotations
from dataclasses import dataclass
import struct

__all__ = ["MMWave_ProfileComCfg", "WIRE_SIZE"]

# Struct format: little-endian, packed
_FMT = struct.Struct("<BBBHfBB")
WIRE_SIZE = _FMT.size  # 11

def hexdump(b: bytes) -> str:
    return " ".join(f"{x:02X}" for x in b)

@dataclass
class MMWave_ProfileComCfg:
    digOutputSampRate: int
    digOutputBitsSel: int
    dfeFirSel: int
    numOfAdcSamples: int
    chirpRampEndTimeus: float
    chirpRxHpfSel: int
    chirpTxMimoPatSel: int

    def to_bytes(self) -> bytes:
        # Mask integers to their field widths; float converted explicitly
        return _FMT.pack(
            self.digOutputSampRate & 0xFF,
            self.digOutputBitsSel  & 0xFF,
            self.dfeFirSel         & 0xFF,
            self.numOfAdcSamples   & 0xFFFF,
            float(self.chirpRampEndTimeus),
            self.chirpRxHpfSel     & 0xFF,
            self.chirpTxMimoPatSel & 0xFF,
        )

    @classmethod
    def from_bytes(cls, b: bytes) -> "MMWave_ProfileComCfg":
        if len(b) != _FMT.size:
            raise ValueError(f"Expected { _FMT.size } bytes, got {len(b)}")
        v = _FMT.unpack(b)
        return cls(*v)
