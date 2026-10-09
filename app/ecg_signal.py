"""Streaming filter chains and experimental ECG/respiration peak metrics.

Per-channel high-pass, optional low-pass and ECG notch are independently
configurable. Detection thresholds/peak selection are not extra waveform
filters. Raw codes are never modified. Sample indices for both channels use
the 500 Hz ECG clock.
"""
from collections import deque
from dataclasses import dataclass
import math
from statistics import median, stdev

import numpy as np
from scipy.signal import find_peaks
from ecg_filters import FilterSettings, FilterPipeline, HighPass

ECG_SPS = 500
RESP_SPS = 50
ADC_NEG = 1 << 23
ADC_POS = ADC_NEG - 1
NEAR_RAIL = 0x7FFF00
LSB_CH1_UV = 2 * 2.42 / (4 * (1 << 24)) * 1e6
LSB_CH2_UV = 2 * 2.42 / (6 * (1 << 24)) * 1e6


@dataclass(frozen=True)
class Sample:
    index: int
    raw: int
    hp: float
    valid: bool
    segment: int
    filtered: float | None = None
    filter_revision: int = 0
    lsb_uv: float | None = None

    @property
    def processed(self):
        return self.hp if self.filtered is None else self.filtered


class Channel:
    def __init__(self, fs, cutoff, warmup=2.0, settings=None):
        self.fs = fs
        self.lsb_uv = None
        self.stride = ECG_SPS // fs
        self.filter = FilterPipeline(fs, settings or FilterSettings(
            hp_hz=cutoff, lp_hz=40.0 if fs == ECG_SPS else 2.0))
        self.filter_revision = 0
        self.samples = deque(maxlen=fs * 60 + 1)
        self.segment = 0
        self.last_index = None
        self.input_valid = False
        self.started = None
        self.warmup = round(warmup * ECG_SPS)

    def configure(self, settings):
        replacement = FilterPipeline(self.fs, settings)
        if settings == self.filter.settings:
            return False
        self.filter = replacement
        self.filter_revision += 1
        self.interrupt()
        return True

    def interrupt(self):
        self.filter.reset()
        self.segment += 1
        self.input_valid = False
        self.started = None

    def add(self, index, raw, valid=True):
        if self.last_index is not None and index - self.last_index != self.stride:
            self.interrupt()
        source_valid = valid and -NEAR_RAIL < raw < NEAR_RAIL
        if not source_valid:
            if self.input_valid:
                self.interrupt()
            hp = float(raw) if self.filter.hp is None else math.nan
            filtered = float(raw) if not self.filter.stages else math.nan
            usable = False
        else:
            if not self.input_valid:
                self.filter.reset()
                self.started = index
                self.input_valid = True
            hp, filtered = self.filter.step(raw)
            usable = index - self.started >= self.warmup
        self.last_index = index
        sample = Sample(index, raw, hp, usable, self.segment, filtered, self.filter_revision, self.lsb_uv)
        self.samples.append(sample)
        return sample

    def usable_tail(self, seconds):
        if not self.samples or not self.samples[-1].valid:
            return []
        last = self.samples[-1]
        low = last.index - seconds * ECG_SPS
        out = []
        for sample in reversed(self.samples):
            if not sample.valid or sample.segment != last.segment or sample.index < low:
                break
            out.append(sample)
        return out[::-1]


@dataclass(frozen=True)
class Interval:
    index: int
    milliseconds: float
    valid: bool
    segment: int


class PeakTracker:
    """Confirmed peaks with a sample-clock watermark and contiguous NN pairs.

    Simple research detector, not a medical/arrhythmia classifier. Poor data
    breaks an analysis segment rather than inventing beats across the gap.
    """
    def __init__(self, kind):
        self.kind = kind
        self.peaks = deque(maxlen=2048)
        self.intervals = deque(maxlen=2048)
        self.total = 0
        self.segment = None
        self.watermark = -1
        self.previous = None
        self.polarity = None

    def restart(self):
        # Preserve cumulative counts, but never mix timing from different filters.
        self.peaks.clear()
        self.intervals.clear()
        self.segment = None
        self.watermark = -1
        self.previous = None
        self.polarity = None

    def update(self, channel):
        ecg = self.kind == 'ecg'
        tail = channel.usable_tail(8 if ecg else 60)
        if len(tail) < channel.fs * (2 if ecg else 8):
            return []
        if self.segment != tail[-1].segment:
            self.segment = tail[-1].segment
            self.watermark = tail[0].index - 1
            self.previous = None
            self.polarity = None
        y = np.array([s.processed for s in tail], dtype=float)
        centered = y - np.median(y)
        swing = float(np.percentile(centered, 99.5) - np.percentile(centered, 0.5))
        if swing < 20:
            return []
        mad = float(np.median(np.abs(centered)))
        prominence = max(10.0, swing * (0.18 if ecg else 0.25),
                         3.0 * mad if ecg else 0.0)
        candidates = {}
        for polarity in (1, -1):
            pos, props = find_peaks(centered * polarity, prominence=prominence,
                                    distance=round(channel.fs * (0.28 if ecg else 1.0)),
                                    width=(channel.fs * (0.008 if ecg else 0.20),
                                           channel.fs * (0.18 if ecg else 15.0)))
            # Largest prominences choose the dominant QRS/respiration polarity.
            score = float(np.mean(sorted(props['prominences'], reverse=True)[:5])) if len(pos) else 0.0
            candidates[polarity] = (pos, score)
        if self.polarity is None:
            self.polarity = max(candidates, key=lambda p: candidates[p][1])
        positions = candidates[self.polarity][0]
        confirmed = tail[-1].index - round(ECG_SPS * (0.12 if ecg else 3.0))
        events = []
        for position in positions:
            sample = tail[position]
            if not self.watermark < sample.index <= confirmed:
                continue
            if self.previous is not None and sample.index - self.previous.index < ECG_SPS * (0.28 if ecg else 1.0):
                continue
            interval = None
            if self.previous is not None:
                ms = (sample.index - self.previous.index) * 1000 / ECG_SPS
                valid = (300 <= ms <= 2000) if ecg else (1000 <= ms <= 15000)
                if ecg:
                    recent = [i.milliseconds for i in list(self.intervals)[-5:]
                              if i.valid and i.segment == self.segment]
                    if len(recent) >= 3:
                        ref = median(recent)
                        valid = valid and abs(ms - ref) <= max(150, ref * 0.25)
                interval = Interval(sample.index, ms, valid, self.segment)
                self.intervals.append(interval)
            self.peaks.append(sample)
            self.total += 1
            self.previous = sample
            events.append((sample, interval))
        self.watermark = max(self.watermark, confirmed)
        return events

    def metrics(self, latest_index, source_ok=True):
        cutoff = latest_index - 60 * ECG_SPS
        intervals = [i for i in self.intervals if i.index >= cutoff]
        good = [i for i in intervals if i.valid]
        result = {'total_peaks': self.total, 'polarity': self.polarity,
                  'valid_intervals': len(good), 'rejected_intervals': len(intervals) - len(good)}
        # Do NOT concatenate NN over a rejected RR or a disconnected segment.
        pairs = [(a.milliseconds, b.milliseconds) for a, b in zip(intervals, intervals[1:])
                 if a.valid and b.valid and a.segment == b.segment]
        current = [i for i in intervals if i.segment == self.segment]
        recent_good = [i for i in current if i.valid]
        age = (latest_index - self.previous.index) / ECG_SPS if self.previous else math.inf
        if self.kind == 'ecg':
            live = (source_ok and age < 3 and current and current[-1].valid)
            result.update(hr_bpm=60000 / current[-1].milliseconds if live else None,
                          avg_hr_bpm=60000 / median([i.milliseconds for i in recent_good[-10:]]) if live else None,
                          rr_ms=current[-1].milliseconds if live else None,
                          sdnn_ms=stdev([i.milliseconds for i in good]) if len(good) >= 2 and source_ok else None,
                          rmssd_ms=math.sqrt(sum((b-a)**2 for a, b in pairs) / len(pairs)) if pairs and source_ok else None,
                          pnn50_pct=100 * sum(abs(b-a) > 50 for a, b in pairs) / len(pairs) if pairs and source_ok else None,
                          nn_valid=len(good), nn_rejected=len(intervals) - len(good))
        else:
            live = source_ok and recent_good and age < max(15, 2 * recent_good[-1].milliseconds / 1000)
            result.update(resp_bpm=60000 / median([i.milliseconds for i in recent_good[-5:]]) if live else None,
                          resp_period_s=recent_good[-1].milliseconds / 1000 if live else None)
        return result


def adc_usage(minimum, maximum):
    """Use raw signed ADC extrema, including DC, not high-pass amplitudes."""
    usage = min(100.0, max(max(0, maximum) / ADC_POS, abs(min(0, minimum)) / ADC_NEG) * 100)
    return usage, max(0.0, 100 - usage)
