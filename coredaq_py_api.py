# coredaq.py (mV-only)
# High-level driver for coreDAQ_LOG (CDC/serial)
#
# pip install pyserial
#
# Example:
#   from coredaq import CoreDAQ
#   with CoreDAQ("COM7") as daq:
#       print(daq.idn())
#       mv = daq.snapshot_mV(n_frames=8)
#       print(mv)
#       ch = daq.transfer_frames_mV(frames=100_000)

import time
import struct,sys
import threading
from typing import List
from array import array
import serial
import serial.tools.list_ports
import warnings



class CoreDAQError(Exception):
    pass


class CoreDAQ:
    # AD7606 bipolar ±5V int16 signed
    FS_VOLTS = 5.0
    CODES_PER_FS = 32768.0

    def __init__(self, port: str, timeout: float = 0.2):
        # Finite timeout is mandatory to avoid “hang forever” on Windows
        self._ser = serial.Serial(
            port=port,
            baudrate=115200,
            timeout=timeout,
            write_timeout=0.5,
        )
        self._lock = threading.Lock()

        # Robust XFER knobs
        self._xfer_stall_timeout_s = 3.0
        self._xfer_read_slice_max = 65536

        # If AD7606 wraps negative at clip, clamp to near FS
        self._sat_mv = 4900.0

        self._drain()

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

    # ---------- Low-level helpers ----------
    def _drain(self):
        try:
            self._ser.reset_input_buffer()
        except Exception:
            pass

    def _writeln(self, s: str):
        if not s.endswith("\n"):
            s += "\n"
        self._ser.write(s.encode("ascii"))

    def _readline(self) -> str:
        raw = self._ser.readline()
        if not raw:
            raise CoreDAQError("Device timeout (no line)")
        return raw.decode("ascii", "ignore").strip()

    def _ask(self, cmd: str) -> str:
        with self._lock:
            self._writeln(cmd)
            line = self._readline()
        return line

    @staticmethod
    def _parse_int(s: str) -> int:
        return int(s, 0)

    # ---------- Identity / State ----------
    def idn(self) -> str:
        line = self._ask("IDN?")
        if not line.startswith("OK"):
            raise CoreDAQError(line)
        return line[2:].strip()

    def state_enum(self) -> int:
        line = self._ask("STATE?")
        if not line.startswith("OK"):
            raise CoreDAQError(line)
        return self._parse_int(line[2:].strip())

    def is_data_ready(self) -> bool:
        return self.state_enum() == 4

    def wait_done(self, poll_s: float = 0.25):
        while self.state_enum() != 4:
            time.sleep(poll_s)

    # ---------- Snapshot (mV only) ----------
    def snapshot_mV(
        self,
        n_frames: int = 1,
        timeout_s: float = 1.0,
        poll_hz: float = 200.0,
    ) -> List[int]:
        """
        SNAP <N> arms snapshot averaging over N frames.
        SNAP? returns:
            OK <mV0> <mV1> <mV2> <mV3>
        or BUSY
        """
        if n_frames <= 0:
            raise ValueError("n_frames must be > 0")

        line = self._ask(f"SNAP {n_frames}")
        if not line.startswith("OK"):
            raise CoreDAQError(f"SNAP arm failed: {line}")

        t0 = time.time()
        sleep_s = 1.0 / max(1.0, float(poll_hz))

        while True:
            line = self._ask("SNAP?")
            if line.startswith("BUSY"):
                if (time.time() - t0) > timeout_s:
                    raise CoreDAQError("Snapshot timeout")
                time.sleep(sleep_s)
                continue

            if not line.startswith("OK"):
                raise CoreDAQError(f"SNAP? failed: {line}")

            payload = line[2:].strip()
            parts = payload.split()
            if len(parts) != 4:
                raise CoreDAQError(f"Unexpected SNAP? payload: {payload!r}")

            try:
                mv = [int(parts[i]) for i in range(4)]
            except Exception as e:
                raise CoreDAQError(f"Bad SNAP? integers: {payload!r}") from e

            # Clamp negative wrap
   

            return mv

    # ---------- Acquisition ----------
    def acq_arm(self, frames: int):
        line = self._ask(f"ACQ ARM {frames}")
        if not line.startswith("OK"):
            raise CoreDAQError(line)

    def acq_start(self):
        line = self._ask("ACQ START")
        if not line.startswith("OK"):
            raise CoreDAQError(line)

    def frames_left(self) -> int:
        line = self._ask("LEFT?")
        if not line.startswith("OK"):
            raise CoreDAQError(line)
        return self._parse_int(line[2:].strip())

    def stream_status(self) -> str:
        line = self._ask("STREAM?")
        if not line.startswith("OK"):
            raise CoreDAQError(line)
        return line[2:].strip()

    def sdram_addr(self) -> int:
        line = self._ask("ADDR?")
        if not line.startswith("OK"):
            raise CoreDAQError(line)
        return self._parse_int(line[2:].strip())

    # ---------- Robust binary receive ----------
    def _read_exactly(self, nbytes: int) -> bytearray:
        ser = self._ser
        buf = bytearray(nbytes)
        mv = memoryview(buf)
        got = 0
        last_progress = time.time()

        while got < nbytes:
            want = min(nbytes - got, self._xfer_read_slice_max)
            r = ser.readinto(mv[got : got + want])
            if r and r > 0:
                got += r
                last_progress = time.time()
            else:
                if (time.time() - last_progress) > self._xfer_stall_timeout_s:
                    raise CoreDAQError(f"XFER stalled at {got}/{nbytes} bytes")
                time.sleep(0.0005)

        return buf

    # ---------- Bulk transfer (legacy protocol) ----------
    def transfer_frames_raw(self, frames: int) -> List[List[float]]:
        """
        Host -> Dev:  XFER <bytes>
        Dev  -> Host: OK ...
                    <binary payload>

        Returns: [ch1, ch2, ch3, ch4] in mV (0.1 mV resolution)
        """
        if frames <= 0:
            raise ValueError("frames must be > 0")

        ser = self._ser
        bytes_needed = frames * 4 * 2  # 4 channels, int16 each

        t0 = time.time()
        with self._lock:
            # Clear any leftovers (ASCII responses etc.)
            ser.reset_input_buffer()

            self._writeln(f"XFER {bytes_needed}")
            ser.flush()

            line = self._readline()
            if not line.startswith("OK"):
                raise CoreDAQError(f"XFER refused: {line}")

            # Read exactly bytes_needed (chunked for Windows stability)
            buf = bytearray(bytes_needed)
            mv = memoryview(buf)
            got = 0
            chunk = 262144  # 256k; good default for Win/mac/linux
            while got < bytes_needed:
                r = ser.read(min(chunk, bytes_needed - got))
                if not r:
                    raise TimeoutError(f"USB read timeout at {got}/{bytes_needed} bytes")
                mv[got:got+len(r)] = r
                got += len(r)

        dt = time.time() - t0
        if dt > 0:
            print(f"[CoreDAQ] Received {bytes_needed} bytes in {dt:.3f}s → {(bytes_needed/1e6/dt):.2f} MB/s")

        # Fast parse as int16 little-endian stream
        samples = array('h')
        samples.frombytes(buf)
        if sys.byteorder != "little":
            samples.byteswap()

        # Convert to mV (0.1 mV/LSB). If you want raw ADC codes, set scale=1.0
        scale = 1
        ch1 = [v * scale for v in samples[0::4]]
        ch2 = [v * scale for v in samples[1::4]]
        ch3 = [v * scale for v in samples[2::4]]
        ch4 = [v * scale for v in samples[3::4]]

        if len(ch1) != frames:
            raise CoreDAQError(f"Parse mismatch: expected {frames} frames, got {len(ch1)}")

        return [ch1, ch2, ch3, ch4]

    # ---------- Trigger ----------
    def trig_arm(self, frames: int, rising: bool = True):
        if frames <= 0:
            raise ValueError("frames must be > 0")
        pol = "R" if rising else "F"    
        line = self._ask(f"TRIGARM {frames} {pol}")
        if not line.startswith("OK"):
            raise CoreDAQError(line)

    # ---------- FREQ / Oversampling (optional) ----------
    def get_freq_hz(self) -> int:
        line = self._ask("FREQ?")
        if not line.startswith("OK"):
            raise CoreDAQError(line)
        return self._parse_int(line[2:].strip())

    def get_oversampling(self) -> int:
        line = self._ask("OS?")
        if not line.startswith("OK"):
            raise CoreDAQError(line)
        return self._parse_int(line[2:].strip())
    
    def _max_freq_for_os(self, os_idx: int) -> int:
        if not (0 <= os_idx <= 7):
            raise ValueError("os_idx must be 0..7")
        base = 100_000
        if os_idx <= 1:
            return base
        return base // (2 ** (os_idx - 1))   # OS2->50k, OS3->25k, ...

    def _best_os_for_freq(self, hz: int) -> int:
        """Return the HIGHEST oversampling index that is still legal for hz."""
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
        """
        Master setting.
        Sets FREQ first. Then adjusts OS downward if the current OS is illegal.
        """
        if hz <= 0 or hz > 100_000:
            raise CoreDAQError("FREQ must be 1..100000 Hz")

        # Set frequency
        st, p = self._ask(f"FREQ {hz}")
        if st != "OK":
            raise CoreDAQError(p)

        # Ensure current OS still legal; if not, reduce OS and warn
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
        """
        Secondary setting.
        If requested OS is illegal for current FREQ, auto-adjust OS to a legal one
        and warn (frequency remains unchanged).
        """
        if not (0 <= os_idx <= 7):
            raise CoreDAQError("OS must be 0..7")

        hz = self.get_freq_hz()

        # If illegal, choose the best OS that still supports the current frequency
        if hz > self._max_freq_for_os(os_idx):
            new_os = self._best_os_for_freq(hz)

            st, p = self._ask(f"OS {new_os}")
            if st != "OK":
                raise CoreDAQError(p)

            warnings.warn(
                f"Requested OS {os_idx} is not valid at {hz} Hz. "
                f"Kept FREQ={hz} Hz and set OS to {new_os}.",
                RuntimeWarning,
                stacklevel=2,
            )
            return

        st, p = self._ask(f"OS {os_idx}")
        if st != "OK":
            raise CoreDAQError(p)

    # ---------- Sensors ----------
    def get_head_temperature_C(self) -> float:
        line = self._ask("TEMP?")
        if not line.startswith("OK"):
            raise CoreDAQError(line)
        return float(line[2:].strip())

    def get_head_humidity(self) -> float:
        line = self._ask("HUM?")
        if not line.startswith("OK"):
            raise CoreDAQError(line)
        return float(line[2:].strip())

    def get_die_temperature_C(self) -> float:
        line = self._ask("DIE_TEMP?")
        if not line.startswith("OK"):
            raise CoreDAQError(line)
        return float(line[2:].strip())

    # ---------- Port discovery ----------
    @staticmethod
    def find() -> List[str]:
        ports: List[str] = []
        for p in serial.tools.list_ports.comports():
            desc = (p.description or "").lower()
            dev = (p.device or "").lower()
            if "usb" in desc or "stm" in desc or "modem" in dev:
                ports.append(p.device)
        return ports