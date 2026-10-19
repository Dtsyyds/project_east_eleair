#!/usr/bin/env python3
"""Measure EMAT thickness from every sample of a TXT/CSV signal.

Examples
--------
python src/measure_thickness.py data/extracted_txt/n10mm.txt --sound-speed 3240
python src/measure_thickness.py --export-first-8192 data
"""

from __future__ import annotations

import argparse
import cmath
import csv
import math
import re
import sys
from pathlib import Path


# 8192 is the requested export length.  Measurement itself uses all samples
# supplied in the input file, including a shorter acquisition.
EXPORT_SAMPLE_COUNT = 8192
DEFAULT_SAMPLE_RATE = 40_000_000.0


def read_signal(path: Path) -> list[float]:
    """Read a one-column TXT file or recorder CSV (index,value,seq)."""
    text = path.read_text(encoding="utf-8-sig", errors="strict")
    if path.suffix.lower() == ".txt":
        # TXT input is a whitespace- or comma-separated stream of bare
        # hexadecimal bytes, for example: 7F 80 FF 00.
        tokens = re.split(r"[\s,]+", text.strip())
        try:
            values = [float(int(token, 16)) for token in tokens if token]
        except ValueError as error:
            raise ValueError("TXT input must contain hexadecimal values such as 7F 80 FF 00") from error
        if any(value < 0 or value > 255 for value in values):
            raise ValueError("TXT hexadecimal values must be bytes from 00 to FF")
        if not values:
            raise ValueError("no numeric samples were found")
        return values
    rows = list(csv.reader(text.splitlines()))
    if not rows:
        raise ValueError("the input file is empty")

    # Recorder CSVs contain a header and an index/value/sequence layout.
    first = [cell.strip().lower() for cell in rows[0]]
    value_column = 1 if len(first) >= 2 and first[0] in {"index", "sample", "time"} else 0
    start_row = 1 if any(re.search(r"[a-zA-Z]", cell) for cell in first) else 0
    values: list[float] = []
    for row in rows[start_row:]:
        if not row:
            continue
        if len(row) <= value_column:
            raise ValueError("a CSV row does not contain a signal-value column")
        token = row[value_column].strip()
        if token:
            values.append(parse_value(token))
    if not values:
        raise ValueError("no numeric samples were found")
    return values


def parse_value(token: str) -> float:
    # Decimal is the normal exported representation.  Permit 0x7f and bare
    # hexadecimal byte tokens such as 7f for recorder TXT files as well.
    try:
        return float(token)
    except ValueError:
        pass
    try:
        return float(int(token, 0))
    except ValueError:
        if re.fullmatch(r"[0-9a-fA-F]+", token):
            return float(int(token, 16))
        raise ValueError(f"invalid sample value: {token!r}") from None


def fft(values: list[complex], inverse: bool = False) -> list[complex]:
    """Radix-2 FFT implemented with the Python standard library only."""
    n = len(values)
    if n == 0 or n & (n - 1):
        raise ValueError("FFT input length must be a power of two")
    result = list(values)
    j = 0
    for i in range(1, n):
        bit = n >> 1
        while j & bit:
            j ^= bit
            bit >>= 1
        j ^= bit
        if i < j:
            result[i], result[j] = result[j], result[i]
    size = 2
    sign = 1 if inverse else -1
    while size <= n:
        step = cmath.exp(sign * 2j * math.pi / size)
        half = size // 2
        for begin in range(0, n, size):
            w = 1 + 0j
            for offset in range(half):
                even = result[begin + offset]
                odd = w * result[begin + offset + half]
                result[begin + offset] = even + odd
                result[begin + offset + half] = even - odd
                w *= step
        size *= 2
    if inverse:
        return [v / n for v in result]
    return result


def next_power_of_two(value: int) -> int:
    return 1 << (value - 1).bit_length()


def moving_average(values: list[float], window: int = 21) -> list[float]:
    half = window // 2
    prefix = [0.0]
    for value in values:
        prefix.append(prefix[-1] + value)
    return [
        (prefix[min(len(values), i + half + 1)] - prefix[max(0, i - half)])
        / (min(len(values), i + half + 1) - max(0, i - half))
        for i in range(len(values))
    ]


def local_peaks(values: list[float], low: int, high: int) -> list[int]:
    # The final element cannot be a local maximum because it has no right
    # neighbour, so the range deliberately stops before ``len(values) - 1``.
    return [i for i in range(max(1, low), min(len(values) - 1, high))
            if values[i] >= values[i - 1] and values[i] > values[i + 1]]


def estimate_interval(envelope: list[float], min_lag: int, max_lag: int) -> int:
    centered = [v - sum(envelope) / len(envelope) for v in envelope]
    # Zero-padding to at least 2N-1 makes this a linear autocorrelation,
    # rather than a circular one, for any input length.
    fft_length = next_power_of_two(2 * len(centered) - 1)
    spectrum = fft([complex(v) for v in centered] + [0j] * (fft_length - len(centered)))
    correlation = fft([v * v.conjugate() for v in spectrum], inverse=True)
    ac = [v.real for v in correlation]
    candidates = local_peaks(ac, min_lag, max_lag)
    if not candidates:
        return max(range(min_lag, max_lag + 1), key=ac.__getitem__)
    return max(candidates, key=ac.__getitem__)


def select_echo_peaks(envelope: list[float], interval: int) -> list[int]:
    maximum = max(envelope)
    candidates = local_peaks(envelope, 0, len(envelope) - 1)
    candidates = [p for p in candidates if envelope[p] >= maximum * 0.15]
    min_distance = max(1, round(interval * 0.70))
    # Highest-first non-maximum suppression gives the same intent as the
    # MATLAB MinPeakDistance setting without a SciPy dependency.
    chosen: list[int] = []
    for peak in sorted(candidates, key=envelope.__getitem__, reverse=True):
        if all(abs(peak - other) >= min_distance for other in chosen):
            chosen.append(peak)
    return sorted(chosen)


def analyse(
    samples: list[float], sound_speed: float, sample_rate: float, adc_zero: float, start_sample: int = 200
) -> dict[str, object]:
    """Calculate thickness and retain the signals needed for a diagnostic plot."""
    if start_sample < 1:
        raise ValueError("start sample must be at least 1")
    samples = samples[start_sample - 1:]
    if len(samples) < 3:
        raise ValueError(f"at least 3 samples are required after sample {start_sample}")
    signal = [v - adc_zero for v in samples]
    fft_length = next_power_of_two(len(signal))
    spectrum = fft([complex(v) for v in signal] + [0j] * (fft_length - len(signal)))
    half = fft_length // 2
    analytic_filter = [1.0] + [2.0] * (half - 1) + [1.0] + [0.0] * (half - 1)
    analytic = fft([v * h for v, h in zip(spectrum, analytic_filter)], inverse=True)[:len(signal)]
    envelope = moving_average([abs(v) for v in analytic])
    if max(envelope) <= 10:
        raise ValueError("no valid echo was detected")

    mm_per_sample = sound_speed / (2.0 * sample_rate) * 1000.0
    min_lag = max(1, round(1.5 / mm_per_sample))
    max_lag = min(len(signal) - 1, round(250.0 / mm_per_sample))
    if min_lag >= max_lag:
        raise ValueError("the sound speed/sample rate gives an invalid search range")
    estimated = estimate_interval(envelope, min_lag, max_lag)
    peaks = select_echo_peaks(envelope, estimated)
    gaps = [b - a for a, b in zip(peaks, peaks[1:])]
    inliers = [gap for gap in gaps if abs(gap - estimated) <= estimated * 0.25]
    final_interval = sorted(inliers)[len(inliers) // 2] if inliers else estimated
    return {
        "thickness_mm": final_interval * mm_per_sample,
        "signal": signal,
        "envelope": envelope,
        "echo_peaks": peaks,
        "sample_indices": list(range(start_sample, start_sample + len(signal))),
        "estimated_interval": estimated,
        "final_interval": final_interval,
    }


def export_first_samples(source_dir: Path, output_dir: Path) -> int:
    output_dir.mkdir(parents=True, exist_ok=True)
    files = sorted(source_dir.glob("*.csv")) + sorted(source_dir.glob("*.txt"))
    files = [p for p in files if p.parent != output_dir]
    if not files:
        raise ValueError(f"no CSV or TXT files found in {source_dir}")
    short_files: list[str] = []
    for source in files:
        values = read_signal(source)
        extracted = values[:EXPORT_SAMPLE_COUNT]
        if len(extracted) < EXPORT_SAMPLE_COUNT:
            short_files.append(f"{source.name} ({len(extracted)})")
        target = output_dir / f"{source.stem}.txt"
        if any(value != round(value) or value < 0 or value > 255 for value in extracted):
            raise ValueError(f"{source.name} contains values outside the hexadecimal-byte range 00 to FF")
        target.write_text(" ".join(f"{round(value):02X}" for value in extracted) + "\n", encoding="utf-8", newline="\n")
        print(f"Exported {len(extracted)} samples: {target}")
    if short_files:
        print("Warning: source files shorter than 8192 were exported in full: " + ", ".join(short_files), file=sys.stderr)
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Measure EMAT thickness using every sample in the input file.")
    parser.add_argument("input", type=Path, nargs="?", help="input TXT or CSV signal file")
    parser.add_argument("--sound-speed", type=float, help="material sound speed in m/s (required for measurement)")
    parser.add_argument("--sample-rate", type=float, default=DEFAULT_SAMPLE_RATE, help="sampling rate in Hz (default: 40000000)")
    parser.add_argument("--adc-zero", type=float, default=127.0, help="ADC zero level (default: 127)")
    parser.add_argument("--start-sample", type=int, default=200, help="first sample used for measurement (default: 200)")
    parser.add_argument("--export-first-8192", type=Path, metavar="DATA_DIR", help="export each signal's first samples from this directory")
    parser.add_argument("--output-dir", type=Path, help="directory for --export-first-8192 (default: DATA_DIR/extracted_txt)")
    args = parser.parse_args()
    try:
        if args.export_first_8192:
            output_dir = args.output_dir or args.export_first_8192 / "extracted_txt"
            return export_first_samples(args.export_first_8192, output_dir)
        if args.input is None or args.sound_speed is None:
            parser.error("measurement requires INPUT and --sound-speed")
        if args.sound_speed <= 0 or args.sample_rate <= 0:
            raise ValueError("sound speed and sample rate must be positive")
        analysis = analyse(read_signal(args.input), args.sound_speed, args.sample_rate, args.adc_zero, args.start_sample)
        # Keep stdout machine-readable for callers: successful measurement
        # responses contain only the numeric thickness in millimetres.
        print(f"{analysis['thickness_mm']:.4f}")
        return 0
    except (OSError, UnicodeError, ValueError) as error:
        print(f"Error: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
