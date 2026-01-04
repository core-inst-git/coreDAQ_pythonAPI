# coredaq_py_api.py #v2.0
# High-level driver for coreDAQ
#
# Changelog v2.0:
#   - Auto-detects LINEAR vs LOG front end once at init (HEAD_TYPE?)
#   - Loads correct calibration (CAL table for LINEAR, LOGCAL LUT for LOG)
#   - Guards unsupported APIs (won't send commands that don't apply)
#   - Power conversion uses ADC codes -> volts (±5V, 16-bit), then:
#       LINEAR: slope/intercept in mV/W
#       LOG   : LUT interpolation voltage->log10(P)->P
#
# REQUIREMENTS:
#   pip install pyserial
#
# USAGE EXAMPLE:
#   from coredaq_py_api import CoreDAQ
#   with CoreDAQ("/dev/tty.usbmodemxxxx") as daq:
#       print(daq.idn())
#       print("Frontend:", daq.frontend_type())
#       codes, gains = daq.snapshot_adc(8)
#       print(codes, gains)
#       print(daq.snapshot_W(8))


import serial, time, struct, threading, math, sys, bisect
import serial.tools.list_ports
from array import array
from typing import Optional, Tuple, List, Union
import warnings


class CoreDAQError(Exception):
    pass


Number = Union[int, float]
NumOrSeq = Union[Number, List[Number], Tuple[Number, ...]]


class CoreDAQ:
    # --- Device/ADC constants ---
    ADC_BITS = 16
    ADC_VFS_VOLTS = 5.0  # ±5 V range (full-scale magnitude)
    # For signed 16-bit bipolar ADC codes, LSB_V = (2*Vfs) / 2^bits
    ADC_LSB_VOLTS = (2.0 * ADC_VFS_VOLTS) / (2 ** ADC_BITS)

    # Keep legacy names used in your old code
    FS_VOLTS = ADC_VFS_VOLTS
    CODES_PER_FS = 32768.0  # signed full-scale codes

    NUM_HEADS = 4
    NUM_GAINS = 8

    FRONTEND_LINEAR = "LINEAR"
    FRONTEND_LOG = "LOG"

    # Nominal maximum recommended optical power per gain (watts), UI guidance only
    GAIN_MAX_POWER_W = [
        4e-3,      # G0: 4 mW
        2e-3,      # G1: 2 mW
        800e-6,    # G2: 800 µW
        400e-6,    # G3: 400 µW
        80e-6,     # G4: 80 µW
        40e-6,     # G5: 40 µW
        4e-6,      # G6: 4 µW
        400e-9,    # G7: 400 nW
    ]

    GAIN_LABELS = [
        "4 mW",
        "2 mW",
        "800 µW",
        "400 µW",
        "80 µW",
        "40 µW",
        "4 µW",
        "400 nW",
    ]

    def __init__(self, port: str, timeout: float = 0.05):
        self._ser = serial.Serial(
            port=port,
            baudrate=115200,
            timeout=timeout,
            write_timeout=0.5
        )
        self._lock = threading.Lock()
        self._drain()

        # Detect frontend type ONCE at init
        self._frontend_type: str = self._detect_frontend_type_once()

        # LINEAR calibration tables (always allocated for API stability)
        self._cal_slope = [[0.0 for _ in range(self.NUM_GAINS)] for _ in range(self.NUM_HEADS)]
        self._cal_intercept = [[0.0 for _ in range(self.NUM_GAINS)] for _ in range(self.NUM_HEADS)]

        # Optional per-channel, per-gain software zero in mV
        self._zero_mv = [[0.0 for _ in range(self.NUM_GAINS)] for _ in range(self.NUM_HEADS)]

        # Deadband threshold for treating near-zero as 0 (mV)
        self._mv_zero_threshold = 15.0

        # LOG LUT storage (filled only if FRONTEND_LOG)
        self._loglut_V_V: Optional[List[float]] = None
        self._loglut_log10P: Optional[List[float]] = None
        self._loglut_V_mV: Optional[List[int]] = None
        self._loglut_log10P_Q16: Optional[List[int]] = None

        # Load correct calibration for this device type
        self._load_calibration_for_frontend()

    # ---------- Lifecycle ----------
    def close(self):
        try:
            if self._ser.is_open:
                self._ser.flush()
                self._ser.reset_input_buffer()
                self._ser.reset_output_buffer()
                self._ser.close()
        except Exception:
            pass

    def __enter__(self):
        return self

    def __exit__(self, et, ev, tb):
        self.close()

    # ---------- Low-level IO helpers ----------
    def _drain(self):
        try:
            self._ser.reset_input_buffer()
        except Exception:
            pass

    def _writeln(self, s: str):
        if not s.endswith("\n"):
            s += "\n"
        self._ser.write(s.encode("ascii", errors="ignore"))

    def _readline(self) -> str:
        raw = self._ser.readline()
        if not raw:
            raise CoreDAQError("Device timeout")
        return raw.decode("ascii", "ignore").strip()

    def _ask(self, cmd: str) -> Tuple[str, str]:
        with self._lock:
            self._writeln(cmd)
            line = self._readline()
        if line.startswith("OK"):
            return "OK", line[2:].strip()
        if line.startswith("ERR"):
            return "ERR", line[3:].strip()
        if line.startswith("BUSY"):
            return "BUSY", ""
        return "ERR", line

    @staticmethod
    def _parse_int(s: str) -> int:
        return int(s, 0)

    # ---------- Frontend detection (ONE TIME) ----------
    def _detect_frontend_type_once(self) -> str:
        """
        Detects frontend type exactly once at init.
        Requires firmware command:
          HEAD_TYPE?
        Response:
          OK TYPE=LOG
          OK TYPE=LINEAR
        """
        # Give the port a moment if macOS toggles DTR/open
        time.sleep(0.05)
        self._drain()

        st, p = self._ask("HEAD_TYPE?")
        if st != "OK":
            raise CoreDAQError(f"HEAD_TYPE? failed: {p}")

        txt = p.strip().upper().replace(" ", "")
        if "TYPE=LOG" in txt:
            return self.FRONTEND_LOG
        if "TYPE=LINEAR" in txt:
            return self.FRONTEND_LINEAR
        raise CoreDAQError(f"Unexpected HEAD_TYPE? reply: {p!r}")

    def frontend_type(self) -> str:
        """Return detected frontend type ('LINEAR' or 'LOG')."""
        return self._frontend_type

    def _require_frontend(self, expected: str, feature: str):
        if self._frontend_type != expected:
            raise CoreDAQError(
                f"{feature} not supported on {self._frontend_type} front end (expected {expected})."
            )

    # ---------- Identity ----------
    def idn(self) -> str:
        st, p = self._ask("IDN?")
        if st != "OK":
            raise CoreDAQError(p)
        return p

    # ---------- ADC conversions ----------
    @classmethod
    def adc_code_to_volts(cls, code: Number) -> float:
        return float(code) * cls.ADC_LSB_VOLTS

    @classmethod
    def adc_code_to_mV(cls, code: Number) -> float:
        return cls.adc_code_to_volts(code) * 1e3

    @classmethod
    def adc_codes_to_volts(cls, codes: List[int]) -> List[float]:
        lsb = cls.ADC_LSB_VOLTS
        return [float(c) * lsb for c in codes]

    @classmethod
    def adc_codes_to_mV(cls, codes: List[int]) -> List[float]:
        lsb_mV = cls.ADC_LSB_VOLTS * 1e3
        return [float(c) * lsb_mV for c in codes]

    # ---------- Calibration loading ----------
    def _load_calibration_for_frontend(self):
        if self._frontend_type == self.FRONTEND_LINEAR:
            self._load_linear_calibration()
        elif self._frontend_type == self.FRONTEND_LOG:
            self._load_log_calibration()
        else:
            raise CoreDAQError(f"Unknown frontend type: {self._frontend_type}")

    def _load_linear_calibration(self):
        """
        Query all heads/gains via CAL <head> <gain> and populate:
          self._cal_slope[head-1][gain]     (mV/W)
          self._cal_intercept[head-1][gain] (mV)

        Expects:
          OK H<h> G<g> S=<SLOPE_HEX> I=<INTERCEPT_HEX>
        """
        for head in range(1, self.NUM_HEADS + 1):
            for gain in range(self.NUM_GAINS):
                status, payload = self._ask(f"CAL {head} {gain}")
                if status != "OK":
                    raise CoreDAQError(f"CAL {head} {gain} failed: {payload}")

                parts = payload.split()
                if len(parts) < 4:
                    raise CoreDAQError(f"Unexpected CAL reply: {payload!r}")

                slope_hex = None
                intercept_hex = None
                for token in parts:
                    if token.startswith("S="):
                        slope_hex = token.split("=", 1)[1]
                    elif token.startswith("I="):
                        intercept_hex = token.split("=", 1)[1]

                if slope_hex is None or intercept_hex is None:
                    raise CoreDAQError(f"Missing S= or I= in CAL reply: {payload!r}")

                try:
                    slope_bits = int(slope_hex, 16)
                    intercept_bits = int(intercept_hex, 16)
                    slope = struct.unpack("<f", slope_bits.to_bytes(4, "little"))[0]
                    intercept = struct.unpack("<f", intercept_bits.to_bytes(4, "little"))[0]
                except Exception as e:
                    raise CoreDAQError(f"Failed parsing CAL payload {payload!r}: {e}")

                self._cal_slope[head - 1][gain] = float(slope)
                self._cal_intercept[head - 1][gain] = float(intercept)

    def _load_log_calibration(self):
        """
        Pull log LUT via:
          LOGCAL 1

        Stream:
          OK H1 N=<n_pts> RB=<rec_bytes>
          <binary payload n_pts*RB>
          OK DONE

        Record = little-endian <Hi:
          uint16 V_mV
          int32  log10P_Q16
        """
        with self._lock:
            self._ser.reset_input_buffer()
            self._writeln("LOGCAL 1")

            header = None
            for _ in range(120):
                raw = self._ser.readline()
                if not raw:
                    continue
                line = raw.decode("ascii", "ignore").strip()
                if line.startswith("OK") and (" N=" in line) and (" RB=" in line) and (" H" in line):
                    header = line
                    break

            if not header:
                raise CoreDAQError("LOGCAL header not received")

            parts = header.split()
            try:
                n_pts = int([t for t in parts if t.startswith("N=")][0].split("=", 1)[1])
                rb = int([t for t in parts if t.startswith("RB=")][0].split("=", 1)[1])
            except Exception:
                raise CoreDAQError(f"Malformed LOGCAL header: {header!r}")

            if rb != 6:
                raise CoreDAQError(f"Unexpected LOGCAL RB={rb} (expected 6)")

            payload_len = n_pts * rb
            payload = self._ser.read(payload_len)
            if len(payload) != payload_len:
                raise CoreDAQError(f"Short LOGCAL payload: got {len(payload)} / {payload_len}")

            done_ok = False
            for _ in range(120):
                raw = self._ser.readline()
                if not raw:
                    continue
                line = raw.decode("ascii", "ignore").strip()
                if line == "OK DONE":
                    done_ok = True
                    break
            if not done_ok:
                raise CoreDAQError("LOGCAL missing OK DONE terminator")

        V_mV: List[int] = []
        Q16: List[int] = []
        for i in range(n_pts):
            v, q = struct.unpack_from("<Hi", payload, i * rb)
            V_mV.append(int(v))
            Q16.append(int(q))

        if not V_mV:
            raise CoreDAQError("LOG LUT empty")

        self._loglut_V_mV = V_mV
        self._loglut_log10P_Q16 = Q16
        self._loglut_V_V = [v / 1000.0 for v in V_mV]
        self._loglut_log10P = [q / 65536.0 for q in Q16]

        # sanity
        if len(self._loglut_V_V) != len(self._loglut_log10P):
            raise CoreDAQError("LOG LUT length mismatch after decode")

    # ---------- Public calibration accessors ----------
    def get_cal_slope(self, head: int, gain: int) -> float:
        self._require_frontend(self.FRONTEND_LINEAR, "get_cal_slope")
        if not (1 <= head <= self.NUM_HEADS):
            raise ValueError("head must be 1..4")
        if not (0 <= gain < self.NUM_GAINS):
            raise ValueError("gain must be 0..7")
        return self._cal_slope[head - 1][gain]

    def get_cal_intercept(self, head: int, gain: int) -> float:
        self._require_frontend(self.FRONTEND_LINEAR, "get_cal_intercept")
        if not (1 <= head <= self.NUM_HEADS):
            raise ValueError("head must be 1..4")
        if not (0 <= gain < self.NUM_GAINS):
            raise ValueError("gain must be 0..7")
        return self._cal_intercept[head - 1][gain]

    # ---------- Gain helpers (LINEAR only) ----------
    def gain_max_power_W(self, gain: int) -> float:
        self._require_frontend(self.FRONTEND_LINEAR, "gain_max_power_W")
        if not (0 <= gain < self.NUM_GAINS):
            raise ValueError("gain must be 0..7")
        return self.GAIN_MAX_POWER_W[gain]

    def gain_label(self, gain: int) -> str:
        self._require_frontend(self.FRONTEND_LINEAR, "gain_label")
        if not (0 <= gain < self.NUM_GAINS):
            raise ValueError("gain must be 0..7")
        return self.GAIN_LABELS[gain]

    def all_gain_labels(self) -> List[str]:
        self._require_frontend(self.FRONTEND_LINEAR, "all_gain_labels")
        return list(self.GAIN_LABELS)

    # ---------- LOG conversion (volts -> power) ----------
    def voltage_to_power_W(self, v_volts: NumOrSeq):
        self._require_frontend(self.FRONTEND_LOG, "voltage_to_power_W")
        if self._loglut_V_V is None or self._loglut_log10P is None:
            raise CoreDAQError("LOG LUT not loaded")

        xs = self._loglut_V_V
        ys = self._loglut_log10P

        def interp_one(x: float) -> float:
            # clamp to ends
            if x <= xs[0]:
                return 10.0 ** ys[0]
            if x >= xs[-1]:
                return 10.0 ** ys[-1]

            j = bisect.bisect_left(xs, x)
            # bracket j-1 and j
            x0, x1 = xs[j - 1], xs[j]
            y0, y1 = ys[j - 1], ys[j]
            if x1 == x0:
                y = y0
            else:
                t = (x - x0) / (x1 - x0)
                y = y0 + t * (y1 - y0)
            return 10.0 ** y

        if isinstance(v_volts, (list, tuple)):
            return [interp_one(float(v)) for v in v_volts]
        return float(interp_one(float(v_volts)))

    # ---------- Snapshot ----------
    def snapshot_adc(self, n_frames: int = 1, timeout_s: float = 1.0, poll_hz: float = 200.0):
        """
        MCU returns ADC codes (signed 16-bit) for 4 channels + gains.
        Returns:
          (codes_list[4], gains_list[4])
        """
        st, payload = self._ask(f"SNAP {n_frames}")
        if st != "OK":
            raise CoreDAQError(f"SNAP arm failed: {payload}")

        t0 = time.time()
        sleep_s = 1.0 / poll_hz

        while True:
            st, payload = self._ask("SNAP?")
            if st == "BUSY":
                if (time.time() - t0) > timeout_s:
                    raise CoreDAQError("Snapshot timeout")
                time.sleep(sleep_s)
                continue

            if st != "OK":
                raise CoreDAQError(f"SNAP? failed: {payload}")

            parts = payload.split()
            if len(parts) < 4:
                raise CoreDAQError(f"SNAP? payload too short: {payload}")

            try:
                codes = [int(parts[i]) for i in range(4)]
            except ValueError as e:
                raise CoreDAQError(f"Failed to parse ADC codes from SNAP?: {payload}") from e

            gains = [0, 0, 0, 0]
            for i, part in enumerate(parts):
                if "G=" in part:
                    try:
                        gains[0] = int(part.split("=")[1])
                        gains[1] = int(parts[i + 1])
                        gains[2] = int(parts[i + 2])
                        gains[3] = int(parts[i + 3])
                    except (ValueError, IndexError) as e:
                        raise CoreDAQError(f"Failed to parse gains from SNAP?: {payload}") from e
                    break

            return codes, gains

    def snapshot_volts(self, n_frames: int = 1, timeout_s: float = 1.0, poll_hz: float = 200.0):
        codes, gains = self.snapshot_adc(n_frames=n_frames, timeout_s=timeout_s, poll_hz=poll_hz)
        v = self.adc_codes_to_volts(codes)
        return v, gains

    def snapshot_mV(self, n_frames: int = 1, timeout_s: float = 1.0, poll_hz: float = 200.0):
        codes, gains = self.snapshot_adc(n_frames=n_frames, timeout_s=timeout_s, poll_hz=poll_hz)
        mv = self.adc_codes_to_mV(codes)
        # keep your old rounding style
        mv = [round(x, 3) for x in mv]
        return mv, gains

    def snapshot_W(self, n_frames: int = 1, timeout_s: float = 1.0, poll_hz: float = 200.0, use_zero: bool = False):
        """
        Returns calibrated optical power (W) for each channel [1..4], based on head type.
          LINEAR: uses slope/intercept (mV/W, mV) and current gains
          LOG   : uses LUT on voltage (V) (gains field is ignored)
        """
        if self._frontend_type == self.FRONTEND_LINEAR:
            mv, gains = self.snapshot_mV(n_frames=n_frames, timeout_s=timeout_s, poll_hz=poll_hz)

            adc_mv_per_lsb = self.ADC_LSB_VOLTS * 1e3
            out: List[float] = []

            for ch in range(4):
                head_idx = ch
                gain = int(gains[ch])

                slope_mV_per_W = float(self._cal_slope[head_idx][gain])
                intercept_mV = float(self._cal_intercept[head_idx][gain])

                if slope_mV_per_W == 0.0:
                    raise CoreDAQError(f"Invalid slope for head {head_idx+1}, gain {gain}")

                mv_corr = float(mv[ch])
                if use_zero:
                    mv_corr -= float(self._zero_mv[head_idx][gain])

                if abs(mv_corr) < float(self._mv_zero_threshold):
                    out.append(0.0)
                    continue

                # rounding based on effective power LSB
                power_lsb = adc_mv_per_lsb / slope_mV_per_W
                decimals = 0 if power_lsb <= 0 else max(0, min(12, round(-math.log10(power_lsb))))

                p_w = (mv_corr - intercept_mV) / slope_mV_per_W
                if p_w < 0.0:
                    p_w = 0.0
                out.append(round(p_w, decimals))

            return out

        if self._frontend_type == self.FRONTEND_LOG:
            mv, _gains = self.snapshot_mV(n_frames=n_frames, timeout_s=timeout_s, poll_hz=poll_hz)
            out: List[float] = []
            for ch in range(4):
                mv_corr = float(mv[ch])
                if abs(mv_corr) < float(self._mv_zero_threshold):
                    out.append(0.0)
                    continue
                v = mv_corr / 1000.0
                out.append(float(self.voltage_to_power_W(v)))
            return out

        raise CoreDAQError(f"Unknown frontend type: {self._frontend_type}")

    # ---------- Autogain (LINEAR only) ----------
    def snapshot_autogain_W(
        self,
        n_frames: int = 5,
        min_mv: float = 0.0,
        max_mv: float = 4500.0,
        max_iters: int = 100,
        settle_s: float = 0.05,
        use_zero: bool = False,
    ):
        self._require_frontend(self.FRONTEND_LINEAR, "snapshot_autogain_W")

        for _ in range(max_iters):
            mv, gains = self.snapshot_mV(n_frames)
            changed = False

            for ch in range(4):
                v = abs(float(mv[ch]))
                g = int(gains[ch])
                head = ch + 1

                if v < min_mv and g < 7:
                    self.set_gain(head, g + 1)
                    changed = True
                elif v > max_mv and g > 0:
                    self.set_gain(head, g - 1)
                    changed = True

            if not changed:
                break

            time.sleep(settle_s)

        mv, gains = self.snapshot_mV(n_frames)
        adc_mv_per_lsb = self.ADC_LSB_VOLTS * 1e3
        power_W: List[float] = []

        for ch in range(4):
            head_idx = ch
            gain = int(gains[ch])
            mv_ch = float(mv[ch])

            slope_mV_per_W = float(self._cal_slope[head_idx][gain])
            intercept_mV = float(self._cal_intercept[head_idx][gain])

            if slope_mV_per_W == 0.0:
                raise CoreDAQError(f"Invalid slope for head {head_idx+1}, gain {gain}")

            mv_corr = mv_ch
            if use_zero:
                mv_corr -= float(self._zero_mv[head_idx][gain])

            if abs(mv_corr) < float(self._mv_zero_threshold):
                power_W.append(0.0)
                continue

            power_lsb = adc_mv_per_lsb / slope_mV_per_W
            decimals = 0 if power_lsb <= 0 else max(0, min(12, round(-math.log10(power_lsb))))

            p_w = (mv_corr - intercept_mV) / slope_mV_per_W
            if p_w < 0.0:
                p_w = 0.0
            power_W.append(round(p_w, decimals))

        return power_W, mv, gains

    # ---------- Gains (LINEAR only) ----------
    def set_gain(self, head: int, value: int) -> None:
        self._require_frontend(self.FRONTEND_LINEAR, "set_gain")

        if head not in (1, 2, 3, 4):
            raise ValueError("head must be 1..4")
        if not (0 <= value <= 7):
            raise ValueError("gain value must be 0..7")

        st, payload = self._ask(f"GAIN {head} {value}")
        if st != "OK":
            raise CoreDAQError(f"GAIN {head} failed: {payload}")

    def get_gains(self) -> Tuple[int, int, int, int]:
        """
        Reads current gains. Only meaningful for LINEAR devices.
        """
        self._require_frontend(self.FRONTEND_LINEAR, "get_gains")

        st, payload = self._ask("GAINS?")
        if st != "OK":
            raise CoreDAQError(f"GAINS? failed: {payload}")

        parts = payload.replace("HEAD", "").replace("=", " ").split()
        try:
            nums = [int(parts[i]) for i in range(1, len(parts), 2)]
            if len(nums) != 4:
                raise ValueError
            return tuple(nums)  # type: ignore[return-value]
        except Exception:
            raise CoreDAQError(f"Unexpected GAINS? payload: '{payload}'")

    def set_gain1(self, value: int): self.set_gain(1, value)
    def set_gain2(self, value: int): self.set_gain(2, value)
    def set_gain3(self, value: int): self.set_gain(3, value)
    def set_gain4(self, value: int): self.set_gain(4, value)

    # ---------- State / acquisition helpers ----------
    def state_enum(self) -> int:
        st, p = self._ask("STATE?")
        if st != "OK":
            raise CoreDAQError(p)
        return self._parse_int(p)
    
        # ============================================================
    # Acquisition control (unified, explicit API)
    # ============================================================

    def arm_acquisition(
        self,
        frames: int,
        use_trigger: bool = False,
        trigger_rising: bool = True,
    ):
        if frames <= 0:
            raise ValueError("frames must be > 0")

        # Arm acquisition buffer
        st, p = self._ask(f"ACQ ARM {frames}")
        if st != "OK":
            raise CoreDAQError(f"ACQ ARM failed: {p}")

        # Optional trigger arm
        if use_trigger:
            pol = "R" if trigger_rising else "F"
            st, p = self._ask(f"TRIGARM {frames} {pol}")
            if st != "OK":
                raise CoreDAQError(f"TRIGARM failed: {p}")

    def start_acquisition(self):
        """
        Software-start acquisition.
        Safe to call even if trigger-armed (firmware will ignore).
        """
        st, p = self._ask("ACQ START")
        if st != "OK":
            raise CoreDAQError(f"ACQ START failed: {p}")

    def stop_acquisition(self):
        st, p = self._ask("ACQ STOP")
        if st != "OK":
            raise CoreDAQError(f"ACQ STOP failed: {p}")

    def acquisition_status(self) -> str:
        """
        Returns firmware stream status, e.g.:
          'STREAMING', 'IDLE'
        """
        st, p = self._ask("STREAM?")
        if st != "OK":
            raise CoreDAQError(p)
        return p

    def frames_remaining(self) -> int:
        st, p = self._ask("LEFT?")
        if st != "OK":
            raise CoreDAQError(p)
        return self._parse_int(p)

    def wait_for_completion(
        self,
        poll_s: float = 0.25,
        timeout_s: float | None = None,
    ):
        READY_STATE = 4  # your existing firmware convention
        t0 = time.time()

        while True:
            if self.state_enum() == READY_STATE:
                return

            if timeout_s is not None and (time.time() - t0) > timeout_s:
                raise CoreDAQError("Acquisition timeout")

            time.sleep(poll_s)

    # ---------- Bulk transfer (ADC codes) ----------
    def transfer_frames_adc(self, frames: int) -> List[List[int]]:
        """
        Transfers <frames> frames of raw ADC codes.
        Host -> Dev:  XFER <bytes>
        Dev  -> Host: OK ...
                      <binary payload>

        Returns: [ch1_codes, ch2_codes, ch3_codes, ch4_codes] each length=frames
        """
        if frames <= 0:
            raise ValueError("frames must be > 0")

        ser = self._ser
        bytes_needed = frames * 4 * 2  # 4 channels, int16 each
        time.sleep(0.05)

        with self._lock:
            ser.reset_input_buffer()
            self._writeln(f"XFER {bytes_needed}")
            ser.flush()

            line = self._readline()
            if not line.startswith("OK"):
                raise CoreDAQError(f"XFER refused: {line}")

            buf = bytearray(bytes_needed)
            mv = memoryview(buf)
            got = 0
            chunk = 262144
            while got < bytes_needed:
                r = ser.read(min(chunk, bytes_needed - got))
                if not r:
                    raise TimeoutError(f"USB read timeout at {got}/{bytes_needed} bytes")
                mv[got:got + len(r)] = r
                got += len(r)

        samples = array('h')
        samples.frombytes(buf)
        if sys.byteorder != "little":
            samples.byteswap()

        ch1 = list(samples[0::4])
        ch2 = list(samples[1::4])
        ch3 = list(samples[2::4])
        ch4 = list(samples[3::4])

        if len(ch1) != frames:
            raise CoreDAQError(f"Parse mismatch: expected {frames} frames, got {len(ch1)}")

        return [ch1, ch2, ch3, ch4]

    # Backward-compatible alias (your old name returned codes already)
    def transfer_frames_raw(self, frames: int) -> List[List[int]]:
        return self.transfer_frames_adc(frames)

    def transfer_frames_volts(self, frames: int) -> List[List[float]]:
        ch = self.transfer_frames_adc(frames)
        lsb = self.ADC_LSB_VOLTS
        return [[float(x) * lsb for x in lst] for lst in ch]

    def transfer_frames_mV(self, frames: int) -> List[List[float]]:
        ch = self.transfer_frames_adc(frames)
        lsb_mV = self.ADC_LSB_VOLTS * 1e3
        return [[float(x) * lsb_mV for x in lst] for lst in ch]

    def transfer_frames_W(self, frames: int, use_zero: bool = False) -> List[List[float]]:
        """
        Transfers frames and converts to optical power in watts per channel.

        LINEAR:
          - uses current GAINS? (assumes fixed during acquisition)
          - applies slope/intercept in mV
        LOG:
          - converts ADC code -> volts -> LUT -> watts
        """
        if frames <= 0:
            raise ValueError("frames must be > 0")

        if self._frontend_type == self.FRONTEND_LINEAR:
            mv_ch = self.transfer_frames_mV(frames)
            gains = self.get_gains()  # requires LINEAR

            adc_mv_per_lsb = self.ADC_LSB_VOLTS * 1e3
            power_ch = [[], [], [], []]

            for ch in range(4):
                head_idx = ch
                gain = int(gains[ch])

                slope_mV_per_W = float(self._cal_slope[head_idx][gain])
                intercept_mV = float(self._cal_intercept[head_idx][gain])

                if slope_mV_per_W == 0.0:
                    raise CoreDAQError(f"Invalid slope for head {head_idx+1}, gain {gain}")

                power_lsb = adc_mv_per_lsb / slope_mV_per_W
                decimals = 0 if power_lsb <= 0 else max(0, min(12, round(-math.log10(power_lsb))))

                out_list = power_ch[ch]
                for mv_val in mv_ch[ch]:
                    mv_corr = float(mv_val)
                    if use_zero:
                        mv_corr -= float(self._zero_mv[head_idx][gain])

                    if abs(mv_corr) < float(self._mv_zero_threshold):
                        out_list.append(0.0)
                        continue

                    p_w = (mv_corr - intercept_mV) / slope_mV_per_W
                    if p_w < 0.0:
                        p_w = 0.0
                    out_list.append(round(p_w, decimals))

            return power_ch

        if self._frontend_type == self.FRONTEND_LOG:
            v_ch = self.transfer_frames_volts(frames)
            power_ch = [[], [], [], []]
            for ch in range(4):
                out_list = power_ch[ch]
                for v in v_ch[ch]:
                    mv_equiv = v * 1e3
                    if abs(mv_equiv) < float(self._mv_zero_threshold):
                        out_list.append(0.0)
                    else:
                        out_list.append(float(self.voltage_to_power_W(v)))
            return power_ch

        raise CoreDAQError(f"Unknown frontend type: {self._frontend_type}")

    # ---------- Misc / settings ----------
    def i2c_refresh(self) -> None:
        st, payload = self._ask("I2C REFRESH")
        if st != "OK":
            raise CoreDAQError(f"I2C REFRESH failed: {payload}")

    def get_oversampling(self) -> int:
        st, p = self._ask("OS?")
        if st != "OK":
            raise CoreDAQError(p)
        return self._parse_int(p)

    def get_freq_hz(self) -> int:
        st, p = self._ask("FREQ?")
        if st != "OK":
            raise CoreDAQError(p)
        return self._parse_int(p)

    def _max_freq_for_os(self, os_idx: int) -> int:
        if not (0 <= os_idx <= 7):
            raise ValueError("os_idx must be 0..7")
        base = 100_000
        if os_idx <= 1:
            return base
        return base // (2 ** (os_idx - 1))

    def _best_os_for_freq(self, hz: int) -> int:
        if hz <= 0:
            raise ValueError("hz must be > 0")
        if hz > 100_000:
            raise ValueError("hz must be <= 100000")
        best = 0
        for os_idx in range(0, 8):
            if hz <= self._max_freq_for_os(os_idx):
                best = os_idx
            else:
                break
        return best

    def set_freq(self, hz: int):
        if hz <= 0 or hz > 100_000:
            raise CoreDAQError("FREQ must be 1..100000 Hz")

        st, p = self._ask(f"FREQ {hz}")
        if st != "OK":
            raise CoreDAQError(p)

        cur_os = self.get_oversampling()
        if hz > self._max_freq_for_os(cur_os):
            new_os = self._best_os_for_freq(hz)
            st, p = self._ask(f"OS {new_os}")
            if st != "OK":
                raise CoreDAQError(p)
            warnings.warn(
                f"OS {cur_os} is not valid at {hz} Hz. Auto-adjusted OS to {new_os}.",
                RuntimeWarning,
                stacklevel=2,
            )

    def set_oversampling(self, os_idx: int):
        if not (0 <= os_idx <= 7):
            raise CoreDAQError("OS must be 0..7")

        hz = self.get_freq_hz()
        if hz > self._max_freq_for_os(os_idx):
            new_os = self._best_os_for_freq(hz)
            st, p = self._ask(f"OS {new_os}")
            if st != "OK":
                raise CoreDAQError(p)
            warnings.warn(
                f"Requested OS {os_idx} is not valid at {hz} Hz. Kept FREQ={hz} Hz and set OS={new_os}.",
                RuntimeWarning,
                stacklevel=2,
            )
            return

        st, p = self._ask(f"OS {os_idx}")
        if st != "OK":
            raise CoreDAQError(p)

    # ---------- Sensors ----------
    def get_head_temperature_C(self) -> float:
        with self._lock:
            self._writeln("TEMP?")
            line = self._readline()
        if not line.startswith("OK"):
            raise CoreDAQError(f"TEMP? error: {line}")
        val = line[3:].strip()
        try:
            return float(val)
        except ValueError:
            raise CoreDAQError(f"Bad TEMP format: '{val}'")

    def get_head_humidity(self) -> float:
        with self._lock:
            self._writeln("HUM?")
            line = self._readline()
        if not line.startswith("OK"):
            raise CoreDAQError(f"HUM? error: {line}")
        val = line[3:].strip()
        try:
            return float(val)
        except ValueError:
            raise CoreDAQError(f"Bad HUM format: '{val}'")

    def get_die_temperature_C(self) -> float:
        with self._lock:
            self._writeln("DIE_TEMP?")
            line = self._readline()
        if not line.startswith("OK"):
            raise CoreDAQError(f"DIE_TEMP? error: {line}")
        val = line[3:].strip()
        try:
            return float(val)
        except ValueError:
            raise CoreDAQError(f"Bad DIE_TEMP format: '{val}'")

    # ---------- Port discovery ----------
    @staticmethod
    def find():
        ports = []
        for p in serial.tools.list_ports.comports():
            if "modem" in p.device.lower() or "STM" in p.description:
                ports.append(p.device)
        return ports