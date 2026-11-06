"""
Parser for the MSP v1 replies the Pluto sends back over Wi-Fi.

Your plutocontrol library (pluto_course/plutocontrol/plutocontrol/Pluto.py, write_function) REQUESTS
MSP_RC, MSP_ATTITUDE, MSP_RAW_IMU, MSP_ALTITUDE and MSP_ANALOG in every ~22 ms cycle, but only reads the
replies inside its blocking getters. The adapter reads the reply stream in its own thread with this parser,
so telemetry never blocks the control loop.

Reply frame:  '$' 'M' '>'  size  cmd  payload[size]  checksum      checksum = XOR(size, cmd, payload)

Payloads (MagisV2 io/serial_msp.cpp):
  105 MSP_RC        : uint16 x channelCount  (rcData AFTER the firmware's processing, l.983-987)
  108 MSP_ATTITUDE  : int16 roll [0.1 deg], int16 pitch [0.1 deg], int16 heading [deg]   (l.988-997)
  109 MSP_ALTITUDE  : int32 estimated altitude [cm], int16 vario [cm/s]                   (l.998-1008)
  110 MSP_ANALOG    : uint16 vBatComp [mV], mAmpRaw, mAhDrawn, mAhRemain, uint8 soc, uint8 warning (l.1017-1025)
"""

import struct

MSP_RC = 105
MSP_ATTITUDE = 108
MSP_ALTITUDE = 109
MSP_ANALOG = 110
HEADER = b"$M>"


class MSPStreamParser:

    def __init__(self, max_buffer=8192):
        self.buffer = bytearray()
        self.max_buffer = max_buffer
        self.checksum_errors = 0
        self.frames = 0

    def feed(self, data):
        """Add received bytes; return a list of (cmd, payload_bytes) for every complete, valid frame."""
        if data:
            self.buffer.extend(data)
        if len(self.buffer) > self.max_buffer:
            del self.buffer[:-self.max_buffer]
        out = []
        while True:
            start = self.buffer.find(HEADER)
            if start < 0:
                # keep a possible partial header at the end
                del self.buffer[:max(0, len(self.buffer) - 2)]
                break
            if start > 0:
                del self.buffer[:start]
            if len(self.buffer) < 5:
                break
            size = self.buffer[3]
            cmd = self.buffer[4]
            total = 5 + size + 1
            if len(self.buffer) < total:
                break
            payload = bytes(self.buffer[5:5 + size])
            checksum = size ^ cmd
            for byte in payload:
                checksum ^= byte
            if checksum == self.buffer[5 + size]:
                out.append((cmd, payload))
                self.frames += 1
                del self.buffer[:total]
            else:
                self.checksum_errors += 1
                del self.buffer[:3]          # skip this header, resynchronise
        return out


def decode(cmd, payload):
    """Return a dict for the known commands, or None."""
    try:
        if cmd == MSP_RC and len(payload) >= 8:
            n = len(payload) // 2
            return {"rc": list(struct.unpack("<%dH" % n, payload[:2 * n]))}
        if cmd == MSP_ATTITUDE and len(payload) >= 6:
            roll, pitch, heading = struct.unpack("<hhh", payload[:6])
            return {"roll_deg": roll / 10.0, "pitch_deg": pitch / 10.0, "heading_deg": float(heading)}
        if cmd == MSP_ALTITUDE and len(payload) >= 6:
            altitude, vario = struct.unpack("<ih", payload[:6])
            return {"baro_alt_cm": float(altitude), "vario_cmps": float(vario)}
        if cmd == MSP_ANALOG and len(payload) >= 10:
            vbat, amps, drawn, remain, soc, warn = struct.unpack("<HHHHBB", payload[:10])
            return {"vbat_v": vbat / 1000.0, "current_ma": float(amps), "mah_drawn": float(drawn),
                    "soc_pct": float(soc), "battery_warning": int(warn)}
    except struct.error:
        return None
    return None


def build_reply(cmd, payload):
    """Encode a reply frame (used by the unit tests and the simulator)."""
    size = len(payload)
    checksum = size ^ cmd
    for byte in payload:
        checksum ^= byte
    return HEADER + bytes([size, cmd]) + payload + bytes([checksum])
