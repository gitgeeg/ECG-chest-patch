"""Independent, causal streaming filter stages (no alteration of raw samples).

Defaults preserve HP-only processing. Additional LP/notch stages are opt-in.
Each channel uses its actual sample rate; there is no offline filtfilt or
resampling. Filter order: high-pass -> notch (ECG only in the UI) -> low-pass.
"""
from dataclasses import asdict, dataclass
import math

from scipy.signal import iirnotch


class Biquad:
    """Transposed DF-II, initialized to the steady state of the first input."""
    def __init__(self, b, a, dc_gain):
        self.b, self.a = tuple(b), tuple(a)
        self.dc_gain = dc_gain
        self.reset()

    def reset(self):
        self.z1 = self.z2 = 0.0
        self.initialized = False

    def step(self, x):
        b0, b1, b2 = self.b
        _, a1, a2 = self.a
        if not self.initialized:
            steady = self.dc_gain * x
            self.z1 = steady - b0 * x
            self.z2 = b2 * x - a2 * steady
            self.initialized = True
        y = b0 * x + self.z1
        self.z1 = b1 * x - a1 * y + self.z2
        self.z2 = b2 * x - a2 * y
        return y


def _check_frequency(cutoff, fs):
    if not math.isfinite(fs) or fs <= 0 or not math.isfinite(cutoff) or not 0 < cutoff < fs / 2:
        raise ValueError(f'频率必须大于 0 且小于奈奎斯特频率 {fs / 2:g} Hz')


class HighPass(Biquad):
    """Second-order Butterworth high-pass; constant startup input yields zero."""
    def __init__(self, cutoff, fs):
        _check_frequency(cutoff, fs)
        k = math.tan(math.pi * cutoff / fs)
        n = 1 / (1 + math.sqrt(2) * k + k * k)
        super().__init__((n, -2 * n, n),
                         (1.0, 2 * (k * k - 1) * n, (1 - math.sqrt(2) * k + k * k) * n), 0.0)


class LowPass(Biquad):
    """Second-order Butterworth low-pass; preserves a constant startup level."""
    def __init__(self, cutoff, fs):
        _check_frequency(cutoff, fs)
        k = math.tan(math.pi * cutoff / fs)
        n = 1 / (1 + math.sqrt(2) * k + k * k)
        b0 = k * k * n
        super().__init__((b0, 2 * b0, b0),
                         (1.0, 2 * (k * k - 1) * n, (1 - math.sqrt(2) * k + k * k) * n), 1.0)


class Notch(Biquad):
    """Second-order IIR notch; Q = center frequency / -3 dB bandwidth."""
    def __init__(self, frequency, quality, fs):
        _check_frequency(frequency, fs)
        if not math.isfinite(quality) or quality <= 0:
            raise ValueError('陷波 Q 必须为有限正数')
        # Very low Q can produce an invalid digital bandwidth even if > 0.
        if frequency / quality >= fs / 2:
            raise ValueError('陷波带宽必须小于奈奎斯特频率')
        b, a = iirnotch(frequency, quality, fs=fs)
        super().__init__(b, a, 1.0)


@dataclass(frozen=True)
class FilterSettings:
    hp_enabled: bool = True
    hp_hz: float = 0.5
    lp_enabled: bool = False
    lp_hz: float = 40.0
    notch_enabled: bool = False
    notch_hz: float = 50.0
    notch_q: float = 30.0

    @classmethod
    def defaults(cls, channel):
        if channel == 'ecg':
            return cls()
        if channel == 'resp':
            return cls(hp_hz=0.05, lp_hz=2.0)
        raise ValueError('Unknown channel')

    def validate(self, fs):
        for value in (self.hp_enabled, self.lp_enabled, self.notch_enabled):
            if not isinstance(value, bool):
                raise ValueError('滤波开关必须为布尔值')
        _check_frequency(self.hp_hz, fs)
        _check_frequency(self.lp_hz, fs)
        if self.hp_enabled and self.lp_enabled and self.hp_hz >= self.lp_hz:
            raise ValueError('同时启用时，高通截止频率必须低于低通截止频率')
        if not math.isfinite(self.notch_hz) or not math.isfinite(self.notch_q) or self.notch_q <= 0:
            raise ValueError('陷波中心频率和 Q 必须为有效数值，Q 必须大于 0')
        if self.notch_enabled:
            _check_frequency(self.notch_hz, fs)
            if self.notch_hz / self.notch_q >= fs / 2:
                raise ValueError('陷波带宽必须小于奈奎斯特频率')
        return self

    def to_dict(self):
        return asdict(self)

    def description(self):
        parts = []
        if self.hp_enabled:
            parts.append(f'高通 {self.hp_hz:g} Hz')
        if self.notch_enabled:
            parts.append(f'陷波 {self.notch_hz:g} Hz / Q{self.notch_q:g}')
        if self.lp_enabled:
            parts.append(f'低通 {self.lp_hz:g} Hz')
        return ' → '.join(parts) if parts else '全部旁路（原始值）'


class FilterPipeline:
    def __init__(self, fs, settings):
        self.settings = settings.validate(fs)
        self.hp = HighPass(settings.hp_hz, fs) if settings.hp_enabled else None
        self.notch = Notch(settings.notch_hz, settings.notch_q, fs) if settings.notch_enabled else None
        self.lp = LowPass(settings.lp_hz, fs) if settings.lp_enabled else None
        self.stages = [s for s in (self.hp, self.notch, self.lp) if s is not None]

    def reset(self):
        for stage in self.stages:
            stage.reset()

    def step(self, x):
        hp = self.hp.step(x) if self.hp is not None else float(x)
        result = self.notch.step(hp) if self.notch is not None else hp
        result = self.lp.step(result) if self.lp is not None else result
        return hp, result
