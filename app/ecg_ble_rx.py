#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ESP32-H2 ECG/respiration BLE monitor. UI by default; --cli for a terminal.

CH2: 500 SPS, ECG, default high-pass 0.5 Hz.
CH1: 50 SPS (firmware block averages), respiration, default high-pass 0.05 Hz.
BMI270: integrated 25 Hz raw accelerometer/gyroscope packets and 5 s temperature updates.
Optional per-channel low-pass and ECG notch have true enable/bypass switches.
Raw, high-pass-stage and final processed data are saved into ONE
local-time-named CSV per session under date/ beside this script.
"""
import argparse
import asyncio
from collections import deque
import csv
from datetime import datetime
import json
import math
from pathlib import Path
import struct
import sys
import threading
import time

import ecg_control as ctl
from ecg_filters import FilterSettings
from ecg_signal import (Channel, PeakTracker, ECG_SPS, RESP_SPS, adc_usage,
                        LSB_CH1_UV, LSB_CH2_UV)

DEVICE_NAME = 'ECG-H2'
SVC_UUID = '6cec0000-106f-3e9a-214d-7b5ca1e30001'
CHAR_UUID = '6cec0000-106f-3e9a-214d-7b5ca1e30002'
RESP_DECIM = 10
BLOCK_BYTES = 33
STATUS_STRUCT = struct.Struct('<IIBBHIIiiiiHB')
STATUS_KEYS = ('sample_idx', 'device_ts_us', 'loff_or', 'loff_now', 'saturation_count',
               'dropped_total', 'bad_frames', 'ch1_min', 'ch1_max', 'ch2_min', 'ch2_max',
               'battery_mv', 'reason')
# type 0x05: sequence, device timestamp, BMI sensor time, status, flags,
# six raw signed-16 axes, and the latest temperature in centi-degrees C.
IMU_STRUCT = struct.Struct('<IIIBB6hh')
IMU_KEYS = ('sequence', 'device_ts_us', 'sensor_time', 'status', 'flags',
            'acc_x_raw', 'acc_y_raw', 'acc_z_raw',
            'gyr_x_raw', 'gyr_y_raw', 'gyr_z_raw', 'temp_c100')
SAVE_DIR = Path(__file__).resolve().parent / 'date'
DATA_STALE_S = 2.0
STATUS_STALE_S = 8.0
CSV_COLUMNS = ('record_type local_time sample_index sample_time_s seq device_ts_us flags '
               'ch2_raw ch2_uV ch2_hp_uV ch2_filtered_uV ch2_filter_revision '
               'ch1_raw ch1_uV ch1_hp_uV ch1_filtered_uV ch1_filter_revision ch2_valid ch1_valid '
               'loff_now loff_or saturation_count dropped_total bad_frames '
               'ch1_min ch1_max ch2_min ch2_max battery_mv reason '
               'hr_bpm avg_hr_bpm rr_ms sdnn_ms rmssd_ms pnn50_pct nn_valid nn_rejected '
               'resp_bpm resp_period_s device_revision device_epoch '
               'imu_seq imu_device_ts_us imu_sensor_time imu_status imu_flags '
               'imu_acc_x_raw imu_acc_y_raw imu_acc_z_raw imu_gyr_x_raw imu_gyr_y_raw imu_gyr_z_raw '
               'imu_acc_x_g imu_acc_y_g imu_acc_z_g imu_gyr_x_dps imu_gyr_y_dps imu_gyr_z_dps '
               'imu_temp_c100 imu_temp_c imu_source note ble_hex').split()


def local_now():
    return datetime.now().astimezone()


def crc16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021 if crc & 0x8000 else crc << 1) & 0xFFFF
    return crc


def i24(data):
    return int.from_bytes(data, 'big', signed=True)


class Parser:
    """Incremental parser: notifications may split or coalesce any frame."""
    def __init__(self):
        self.buf = bytearray()
        self.crc_errors = 0
        self.malformed = 0
        self.frames = 0

    def feed(self, chunk):
        self.buf.extend(chunk)
        out = []
        while True:
            start = self.buf.find(b'\xa5\x5a')
            if start < 0:
                self.buf[:] = self.buf[-1:] if self.buf[-1:] == b'\xa5' else b''
                break
            del self.buf[:start]
            if len(self.buf) < 4:
                break
            kind, size = self.buf[2:4]
            if ((kind in (1, 4) and (size < (43 if kind == 1 else 51) or
                    (size - (10 if kind == 1 else 18)) % BLOCK_BYTES)) or
                    (kind == 2 and size != STATUS_STRUCT.size) or
                    (kind == 3 and size != ctl.REPLY_SIZE) or
                    (kind == 5 and size != IMU_STRUCT.size)):
                self.malformed += 1
                del self.buf[:2]
                continue
            total = size + 6
            if len(self.buf) < total:
                break
            body = self.buf[2:4 + size]
            received = int.from_bytes(self.buf[4 + size:total], 'little')
            if crc16(body) != received:
                self.crc_errors += 1
                del self.buf[:2]
                continue
            out.append((kind, bytes(self.buf[4:4 + size])))
            self.frames += 1
            del self.buf[:total]
        return out


class SessionLog:
    """One lazy, exclusively-created file. Timestamps carry local UTC offset."""
    def __init__(self, directory=SAVE_DIR):
        self.directory = Path(directory)
        self.path = None
        self.file = None
        self.writer = None
        self.last_flush = 0.0

    def start(self, started, filter_configuration=None, device_configuration=None):
        if self.file is not None:
            return
        self.directory.mkdir(parents=True, exist_ok=True)
        stamp = started.strftime('%Y-%m-%d_%H-%M-%S_%f')[:-3]
        for suffix in range(10000):
            path = self.directory / (stamp + (f'_{suffix}' if suffix else '') + '.csv')
            try:
                self.file = path.open('x', encoding='utf-8-sig', newline='')
                self.path = path
                break
            except FileExistsError:
                continue
        else:
            raise OSError('Cannot create a unique session file')
        self.writer = csv.DictWriter(self.file, CSV_COLUMNS, extrasaction='ignore')
        self.writer.writeheader()
        self.write('SESSION', started, note={
            'device': DEVICE_NAME, 'ecg_sps': ECG_SPS, 'resp_sps': RESP_SPS,
            'schema_version': 4,
            'device_configuration': device_configuration,
            'imu': {'device': 'BMI270', 'accel_odr_hz': 25, 'accel_range_g': 2,
                    'gyro_odr_hz': 25, 'gyro_range_dps': 2000,
                    'int1_gpio': 10, 'i2c_sda_gpio': 12, 'i2c_scl_gpio': 14,
                    'packet_type': 5},
            'filters': filter_configuration or {
                name: {'revision': 0, 'settings': FilterSettings.defaults(name).to_dict()}
                for name in ('ecg', 'resp')},
            'filter_order': 'high-pass -> notch -> low-pass', 'filter_order_per_stage': 2,
            'vref_V': 2.42,
            'ch1_gain': device_configuration['ch1_gain'] if device_configuration else 4,
            'ch2_gain': device_configuration['ch2_gain'] if device_configuration else 6,
            'ch1_transport': 'mean of 10 firmware samples; timestamp at block end',
            'sample_clock_hz': ECG_SPS, 'local_time': 'PC receive time, not device capture time',
            'battery_gpio': 2, 'battery_divider': '13k upper / 15k lower; Vbat=Vadc*28/15',
            'analysis': 'experimental peaks and 60 s time-domain HRV, not a diagnosis',
        })
        self.flush(force=True)

    def write(self, record_type, timestamp=None, **values):
        if self.writer is None:
            return
        row = {'record_type': record_type, 'local_time': (timestamp or local_now()).isoformat(timespec='milliseconds')}
        for key, value in values.items():
            if isinstance(value, float):
                value = round(value, 6) if math.isfinite(value) else ''
            elif isinstance(value, (dict, list)):
                value = json.dumps(value, ensure_ascii=False, separators=(',', ':'))
            row[key] = value
        self.writer.writerow(row)

    def flush(self, force=False):
        now = time.monotonic()
        if self.file is not None and (force or now - self.last_flush >= 1):
            self.file.flush()
            self.last_flush = now

    def close(self, totals):
        if self.file is not None:
            try:
                self.write('END', note=totals)
                self.flush(force=True)
            finally:
                self.file.close()
                self.file = self.writer = None


def electrode_states(status, fresh, flags=0, loff_sens=0x00, rld_sens=0xAC):
    states = {}
    for name, bit in (('IN1P', 1), ('IN1N', 2), ('IN2P', 4), ('IN2N', 8), ('RLD', 16)):
        enabled = bool(loff_sens & bit) if name != 'RLD' else bool(rld_sens & 0x10)
        if not enabled:
            states[name] = ('未启用检测', '#8292a7')
        elif not fresh or status is None:
            states[name] = ('未知 / 状态过期', '#8292a7')
        elif status['loff_now'] & bit:
            states[name] = ('脱落', '#fb7185')
        elif status['loff_or'] & bit or (name in ('IN2P', 'IN2N') and flags & 1):
            states[name] = ('已连接 / 窗口内脱落', '#fbbf24')
        else:
            states[name] = ('已连接', '#34d399')
    return states


def number(value, digits=0, suffix=''):
    if value is None or not math.isfinite(value):
        return '--'
    return f'{value:.{digits}f}{suffix}'

class Sink:
    """Worker acquires data; locked UI reads and filter updates serialize with it."""
    def __init__(self, directory=SAVE_DIR, filter_settings=None):
        self.lock = threading.RLock()
        self.log = SessionLog(directory)
        self.parser = Parser()
        settings = filter_settings or {}
        ecg_settings = settings.get('ecg', FilterSettings.defaults('ecg'))
        resp_settings = settings.get('resp', FilterSettings.defaults('resp'))
        if resp_settings.notch_enabled:
            raise ValueError('呼吸通道不提供工频陷波；实际采样率仅 50 SPS')
        self.ecg = Channel(ECG_SPS, 0.5, settings=ecg_settings)
        self.resp = Channel(RESP_SPS, 0.05, settings=resp_settings)
        self.ecg.lsb_uv, self.resp.lsb_uv = LSB_CH2_UV, LSB_CH1_UV
        self.control_supported = self.control_known = self.control_busy = False
        self.control_message = '等待连接；远程 ADS 控制需要新固件。'
        self.device_state = self.device_revision = self.device_epoch = None
        self.device_registers = dict(ctl.DEFAULT_REGISTERS)
        self.stream_config_known = True  # legacy firmware uses fixed defaults
        self.unannounced_config = None
        self.r_tracker = PeakTracker('ecg')
        self.b_tracker = PeakTracker('resp')
        self.connection = '准备连接'
        self.connected = False
        self.error = None
        self.next_seq = None
        self.timeline_block = None
        self.origin = None
        self.lost_blocks = self.blocks = self.samples = self.notifications = 0
        self.last_data_at = self.last_status_at = None
        self.last_status = None
        self.last_flags = 0
        self.last_device_ts = None
        self.last_imu = None
        self.last_imu_at = None
        self.imu_samples = 0
        self.last_analysis = -math.inf
        self.pending = deque(maxlen=128)
        self.closed = False

    def device_configuration(self):
        with self.lock:
            return {'known': self.control_known, 'state': self.device_state,
                    'revision': self.device_revision, 'epoch': self.device_epoch,
                    'registers': {ctl.BY_ADDRESS[a].name: v for a, v in self.device_registers.items()},
                    'ch1_gain': ctl.channel_gain(self.device_registers, 4),
                    'ch2_gain': ctl.channel_gain(self.device_registers, 5)}

    def set_control_support(self, supported):
        with self.lock:
            self.control_supported = supported
            self.control_message = '正在读取 ADS 状态…' if supported else '设备无控制特征；旧固件仅支持接收。'
            if supported and not self.control_known:
                self.stream_config_known = False
                self.ecg.lsb_uv = self.resp.lsb_uv = math.nan

    def _record_control_event(self, kind, note):
        if self.closed:
            return
        if self.log.path is None:
            self.pending.append((local_now(), kind, {'note': note}))
        else:
            self.log.write(kind, note=note)

    def set_control_message(self, text, busy=None):
        with self.lock:
            self.control_message = text
            if busy is not None:
                self.control_busy = busy
            self._record_control_event('COMMAND_RESULT', {'message': text})
            self.log.flush()

    def command_sent(self, opcode, request_id, registers):
        with self.lock:
            self.control_message = f'{ctl.OP_NAMES[opcode]}：已发送，等待 ADS 执行确认…'
            self.control_busy = True
            self._record_control_event('COMMAND', {'opcode': opcode, 'request_id': request_id,
                                           'registers': {ctl.BY_ADDRESS[a].name: v for a, v in registers.items()}})
            self.log.flush()

    def control(self, payload, timestamp=None):
        try:
            reply = ctl.decode_reply(payload)
        except ValueError:
            self.parser.malformed += 1
            return None
        with self.lock:
            changed = (not self.control_known or self.device_state != reply['state'] or
                       self.device_revision != reply['revision'] or self.device_epoch != reply['epoch'] or
                       self.device_registers != reply['registers'] or not self.stream_config_known)
            self.device_state, self.device_revision, self.device_epoch = reply['state'], reply['revision'], reply['epoch']
            self.device_registers = dict(reply['registers'])
            self.control_known = self.stream_config_known = True
            self.unannounced_config = None
            self.ecg.lsb_uv = 2.42e6 / (ctl.channel_gain(self.device_registers, 5) * (1 << 23))
            self.resp.lsb_uv = 2.42e6 / (ctl.channel_gain(self.device_registers, 4) * (1 << 23))
            if changed:
                self._break()
                self.r_tracker.restart()
                self.b_tracker.restart()
                self.last_data_at = self.last_status_at = None
                self.last_status = None
                self.last_analysis = -math.inf
            if not self.control_busy:
                self.control_message = f"ADS {ctl.STATE_NAMES[self.device_state]} · 配置版本 {self.device_revision}"
            timestamp = timestamp or local_now()
            self._start(timestamp)
            self.log.write('DEVICE', timestamp, device_revision=self.device_revision, device_epoch=self.device_epoch,
                           note={**reply, 'registers': self.device_configuration()['registers']})
            self.log.flush()
            return reply

    def filter_configuration(self):
        with self.lock:
            return {name: {'revision': channel.filter_revision,
                           'settings': channel.filter.settings.to_dict()}
                    for name, channel in (('ecg', self.ecg), ('resp', self.resp))}

    def get_filter_settings(self):
        with self.lock:
            return {'ecg': self.ecg.filter.settings, 'resp': self.resp.filter.settings}

    def configure_filters(self, ecg=None, resp=None):
        with self.lock:
            proposed = {'ecg': ecg or self.ecg.filter.settings,
                        'resp': resp or self.resp.filter.settings}
            # Validate the full transaction before modifying either channel.
            proposed['ecg'].validate(ECG_SPS)
            proposed['resp'].validate(RESP_SPS)
            if proposed['resp'].notch_enabled:
                raise ValueError('呼吸通道不提供工频陷波；实际采样率仅 50 SPS')
            changes = []
            for name, channel, tracker in (('ecg', self.ecg, self.r_tracker),
                                            ('resp', self.resp, self.b_tracker)):
                if channel.configure(proposed[name]):
                    tracker.restart()
                    changes.append(name)
                    self.log.write('FILTER', note={
                        'channel': name, 'revision': channel.filter_revision,
                        'settings': channel.filter.settings.to_dict(),
                        'previous_last_sample_index': channel.last_index,
                        'applies_to': 'subsequently received samples',
                        'analysis_window_reset': True})
            if changes:
                self.last_analysis = -math.inf
                self.log.flush(force=True)
            return changes

    def set_connection(self, text, connected=False, error=None):
        with self.lock:
            self.connection, self.connected, self.error = text, connected, error
            self.log.write('LINK', note={'state': text, 'connected': connected, 'error': error})
            if not connected:
                self.ecg.interrupt()
                self.resp.interrupt()
            self.log.flush(force=True)

    def _start(self, timestamp):
        if self.log.path is None:
            self.log.start(timestamp, self.filter_configuration(), self.device_configuration())
            self.log.write('LINK', timestamp, note={'state': self.connection, 'connected': self.connected})
            for when, kind, values in self.pending:
                self.log.write(kind, when, **values)
            self.pending.clear()

    def notification(self, chunk):
        timestamp = local_now()
        with self.lock:
            if self.closed:
                return
            self.notifications += 1
            chunk = bytes(chunk)
            if self.log.path is None:
                self.pending.append((timestamp, 'BLE', {'ble_hex': chunk.hex()}))
            else:
                self.log.write('BLE', timestamp, ble_hex=chunk.hex())
            replies = []
            for kind, payload in self.parser.feed(chunk):
                if kind in (1, 4):
                    self.data(payload, timestamp, extended=kind == 4)
                elif kind == 2:
                    self.status(payload, timestamp)
                elif kind == 3:
                    reply = self.control(payload, timestamp)
                    if reply is not None:
                        replies.append(reply)
                elif kind == 5:
                    self.imu(payload, timestamp)
            self.log.flush()
            return replies

    def _break(self):
        self.ecg.interrupt()
        self.resp.interrupt()

    def _sample_time(self, index):
        return (index - self.origin) / ECG_SPS if self.origin is not None else None

    def data(self, payload, timestamp=None, extended=False):
        timestamp = timestamp or local_now()
        header_size = 18 if extended else 10
        if len(payload) < header_size:
            self.parser.malformed += 1
            return
        seq, device_ts, flags, nblk = struct.unpack_from('<IIBB', payload)
        if not 1 <= nblk <= 7 or len(payload) != header_size + nblk * BLOCK_BYTES:
            self.parser.malformed += 1
            return
        with self.lock:
            if self.closed:
                return
            if extended:
                wire_revision, wire_epoch = struct.unpack_from('<II', payload, 10)
                if not self.control_known or (wire_revision, wire_epoch) != (self.device_revision, self.device_epoch):
                    if self.unannounced_config != (wire_revision, wire_epoch):
                        self._break()
                        self.r_tracker.restart()
                        self.b_tracker.restart()
                    self.unannounced_config = (wire_revision, wire_epoch)
                    self.stream_config_known = False
                    self.ecg.lsb_uv = self.resp.lsb_uv = math.nan
                    self.control_message = '收到未确认配置的数据；保留原始值，等待设备配置同步。'
            else:
                wire_revision, wire_epoch = self.device_revision, self.device_epoch
            self._start(timestamp)
            if self.next_seq is None:
                base = seq
                self.origin = base * RESP_DECIM
            else:
                gap = (seq - self.next_seq) & 0xFFFFFFFF
                base = self.timeline_block
                if gap:
                    if gap >= 0x80000000 and not (seq < 1000 and device_ts < (self.last_device_ts or 0)):
                        self.log.write('GAP', timestamp, seq=seq, note='duplicate/out-of-order DATA discarded')
                        return
                    self._break()
                    if gap <= 1000000:
                        self.lost_blocks += gap
                        base += gap
                        note = {'missing_blocks': gap, 'expected_seq': self.next_seq}
                    else:
                        base += 1
                        note = {'device_sequence_reset': True, 'expected_seq': self.next_seq,
                                'wire_seq': seq, 'gap_not_counted_as_billions': True}
                    self.log.write('GAP', timestamp, seq=seq, sample_index=base * RESP_DECIM,
                                   sample_time_s=self._sample_time(base * RESP_DECIM), note=note)
            self.next_seq = (seq + nblk) & 0xFFFFFFFF
            self.timeline_block = base + nblk
            self.last_flags, self.last_device_ts = flags, device_ts
            self.last_data_at = time.monotonic()
            # Filled samples are indistinguishable inside the flagged packet.
            # Conservatively exclude the whole packet from both detectors.
            source_ok = self.stream_config_known and self.device_state not in (ctl.STOPPED, ctl.FAULT)
            ecg_ok = source_ok and not flags & 0x0B and self.device_registers[5] & 0x8F == 0
            resp_ok = source_ok and not flags & 0x0E and self.device_registers[4] & 0x8F == 0
            if self.control_known and self.device_registers[9] & 0xC0 != 0xC0:
                resp_ok = False
            if self.last_status is not None:
                loff = self.last_status['loff_now'] & self.device_registers[7]
                if loff & 0x0C: ecg_ok = False
                if loff & 0x03: resp_ok = False
                if self.device_registers[6] & 0x10 and self.last_status['loff_now'] & 0x10:
                    ecg_ok = resp_ok = False
            for block_no in range(nblk):
                offset = header_size + block_no * BLOCK_BYTES
                raw_ch1 = i24(payload[offset + 30:offset + 33])
                for sample_no in range(RESP_DECIM):
                    index = (base + block_no) * RESP_DECIM + sample_no
                    raw_ch2 = i24(payload[offset + sample_no * 3:offset + sample_no * 3 + 3])
                    e = self.ecg.add(index, raw_ch2, ecg_ok)
                    row = {'sample_index': index, 'sample_time_s': self._sample_time(index),
                           'seq': (seq + block_no) & 0xFFFFFFFF, 'device_ts_us': device_ts,
                           'flags': flags, 'device_revision': wire_revision, 'device_epoch': wire_epoch,
                           'ch2_raw': raw_ch2, 'ch2_uV': raw_ch2 * e.lsb_uv,
                           'ch2_hp_uV': e.hp * e.lsb_uv, 'ch2_filtered_uV': e.processed * e.lsb_uv,
                           'ch2_filter_revision': e.filter_revision, 'ch2_valid': int(e.valid)}
                    if sample_no == RESP_DECIM - 1:
                        r = self.resp.add(index, raw_ch1, resp_ok)
                        row.update(ch1_raw=raw_ch1, ch1_uV=raw_ch1 * r.lsb_uv,
                                   ch1_hp_uV=r.hp * r.lsb_uv, ch1_filtered_uV=r.processed * r.lsb_uv,
                                   ch1_filter_revision=r.filter_revision, ch1_valid=int(r.valid))
                    self.log.write('SAMPLE', timestamp, **row)
            self.blocks += nblk
            self.samples += nblk * RESP_DECIM
            self.analyze(timestamp)
            self.log.flush()

    def status(self, payload, timestamp=None):
        timestamp = timestamp or local_now()
        if len(payload) != STATUS_STRUCT.size:
            self.parser.malformed += 1
            return
        with self.lock:
            if self.closed:
                return
            self._start(timestamp)
            status = dict(zip(STATUS_KEYS, STATUS_STRUCT.unpack(payload)))
            self.last_status = status
            self.last_status_at = time.monotonic()
            self.log.write('STATUS', timestamp, sample_index=status['sample_idx'], **status,
                           note={'battery_available': status['battery_mv'] != 0xFFFF})
            if status['reason'] == 2:
                self._break()
            else:
                loff = status['loff_now'] & self.device_registers[7]
                if loff & 0x0C: self.ecg.interrupt()
                if loff & 0x03: self.resp.interrupt()
                if self.device_registers[6] & 0x10 and status['loff_now'] & 0x10:
                    self._break()
            self.log.flush()

    def imu(self, payload, timestamp=None):
        timestamp = timestamp or local_now()
        if len(payload) != IMU_STRUCT.size:
            self.parser.malformed += 1
            return
        values = dict(zip(IMU_KEYS, IMU_STRUCT.unpack(payload)))
        flags = values['flags']
        temp_valid = bool(flags & 0x02) and values['temp_c100'] != -32768
        sample = {
            **values,
            'temp_valid': temp_valid,
            'temp_c': values['temp_c100'] / 100.0 if temp_valid else None,
            'source': 'IRQ' if flags & 0x01 else 'POLL',
            'acc_g': tuple(v * 2.0 / 32768.0 for v in
                           (values['acc_x_raw'], values['acc_y_raw'], values['acc_z_raw'])),
            'gyr_dps': tuple(v * 2000.0 / 32768.0 for v in
                             (values['gyr_x_raw'], values['gyr_y_raw'], values['gyr_z_raw'])),
        }
        with self.lock:
            if self.closed:
                return
            self._start(timestamp)
            self.last_imu = sample
            self.last_imu_at = time.monotonic()
            self.imu_samples += 1
            self.log.write('IMU', timestamp,
                           imu_seq=values['sequence'],
                           imu_device_ts_us=values['device_ts_us'],
                           imu_sensor_time=values['sensor_time'],
                           imu_status=values['status'], imu_flags=values['flags'],
                           imu_acc_x_raw=values['acc_x_raw'],
                           imu_acc_y_raw=values['acc_y_raw'],
                           imu_acc_z_raw=values['acc_z_raw'],
                           imu_gyr_x_raw=values['gyr_x_raw'],
                           imu_gyr_y_raw=values['gyr_y_raw'],
                           imu_gyr_z_raw=values['gyr_z_raw'],
                           imu_acc_x_g=sample['acc_g'][0],
                           imu_acc_y_g=sample['acc_g'][1],
                           imu_acc_z_g=sample['acc_g'][2],
                           imu_gyr_x_dps=sample['gyr_dps'][0],
                           imu_gyr_y_dps=sample['gyr_dps'][1],
                           imu_gyr_z_dps=sample['gyr_dps'][2],
                           imu_temp_c100=values['temp_c100'] if temp_valid else None,
                           imu_temp_c=sample['temp_c'], imu_source=sample['source'])

    def analyze(self, timestamp=None):
        now = time.monotonic()
        if now - self.last_analysis < 0.25:
            return
        self.last_analysis = now
        for channel, tracker, name in ((self.ecg, self.r_tracker, 'R'), (self.resp, self.b_tracker, 'BREATH')):
            for sample, interval in tracker.update(channel):
                values = {'sample_index': sample.index, 'sample_time_s': self._sample_time(sample.index),
                          'device_revision': self.device_revision, 'device_epoch': self.device_epoch,
                          'note': {'polarity': tracker.polarity, 'segment': sample.segment,
                                   'interval_valid': interval.valid if interval else None}}
                if name == 'R':
                    values.update(ch2_raw=sample.raw, ch2_hp_uV=sample.hp * sample.lsb_uv,
                                  ch2_filtered_uV=sample.processed * sample.lsb_uv,
                                  ch2_filter_revision=sample.filter_revision,
                                  rr_ms=interval.milliseconds if interval else None)
                else:
                    values.update(ch1_raw=sample.raw, ch1_hp_uV=sample.hp * sample.lsb_uv,
                                  ch1_filtered_uV=sample.processed * sample.lsb_uv,
                                  ch1_filter_revision=sample.filter_revision,
                                  resp_period_s=interval.milliseconds / 1000 if interval else None)
                self.log.write(name, timestamp, **values)
        metrics = self._metrics(now)
        self.log.write('ANALYSIS', timestamp, **metrics)

    def _metrics(self, now):
        data_fresh = self.connected and self.last_data_at is not None and now - self.last_data_at < DATA_STALE_S
        result = {}
        for channel, tracker in ((self.ecg, self.r_tracker), (self.resp, self.b_tracker)):
            latest = channel.samples[-1].index if channel.samples else 0
            ok = (self.connected and data_fresh and channel.input_valid and channel.samples
                  and channel.samples[-1].valid and tracker.segment == channel.segment)
            metrics = tracker.metrics(latest, bool(ok))
            result.update({k: v for k, v in metrics.items() if k in CSV_COLUMNS})
        return result

    def snapshot(self, include_samples=True):
        with self.lock:
            now = time.monotonic()
            data_fresh = self.connected and self.last_data_at is not None and now - self.last_data_at < DATA_STALE_S
            status_fresh = (self.connected and self.last_status_at is not None
                            and now - self.last_status_at < STATUS_STALE_S)
            status = dict(self.last_status) if self.last_status is not None else None
            usage = {}
            for name, channel in (('ch1', self.resp), ('ch2', self.ecg)):
                latest = channel.samples[-1].index if channel.samples else 0
                raw = [s.raw for s in reversed(channel.samples) if s.index >= latest - ECG_SPS * 5]
                if raw and data_fresh:
                    lo, hi = min(raw), max(raw)
                    if status_fresh and status:
                        lo, hi = min(lo, status[name + '_min']), max(hi, status[name + '_max'])
                    usage[name] = adc_usage(lo, hi)
                else:
                    usage[name] = None
            battery = status['battery_mv'] if status_fresh and status and status['battery_mv'] != 0xFFFF else None
            imu_fresh = (self.connected and self.last_imu_at is not None
                         and now - self.last_imu_at < DATA_STALE_S)
            return {'device': self.device_configuration(), 'control_supported': self.control_supported,
                    'control_busy': self.control_busy, 'control_message': self.control_message,
                    'filters': self.filter_configuration(), 'connection': self.connection, 'connected': self.connected, 'error': self.error,
                    'data_fresh': data_fresh, 'status_fresh': status_fresh, 'status': status,
                    'imu_fresh': imu_fresh, 'imu': dict(self.last_imu) if self.last_imu else None,
                    'imu_samples': self.imu_samples,
                    'last_flags': self.last_flags, 'battery_mv': battery, 'usage': usage,
                    'electrodes': electrode_states(status, status_fresh and self.device_state not in (ctl.STOPPED, ctl.FAULT),
                                                    self.last_flags, self.device_registers[7], self.device_registers[6]),
                    'metrics': self._metrics(now), 'r_total': self.r_tracker.total,
                    'breath_total': self.b_tracker.total, 'blocks': self.blocks, 'samples': self.samples,
                    'lost_blocks': self.lost_blocks, 'crc_errors': self.parser.crc_errors,
                    'malformed': self.parser.malformed, 'notifications': self.notifications,
                    'path': str(self.log.path) if self.log.path else None,
                    'origin': self.origin or 0,
                    'ecg': list(self.ecg.samples) if include_samples else [],
                    'resp': list(self.resp.samples) if include_samples else [],
                    'r_peaks': list(self.r_tracker.peaks) if include_samples else []}

    def close(self):
        with self.lock:
            if not self.closed:
                self.closed = True
                self.log.close({'ecg_samples': self.samples, 'resp_samples': self.blocks,
                                'lost_blocks': self.lost_blocks, 'crc_errors': self.parser.crc_errors,
                                'r_peaks': self.r_tracker.total, 'breath_peaks': self.b_tracker.total,
                                'imu_samples': self.imu_samples,
                                'malformed': self.parser.malformed, 'notifications': self.notifications})

async def find_device():
    from bleak import BleakScanner

    def matches(device, advertisement):
        name = advertisement.local_name or device.name or ''
        services = {u.lower() for u in (advertisement.service_uuids or [])}
        return name == DEVICE_NAME or SVC_UUID in services

    device = await BleakScanner.find_device_by_filter(matches, timeout=15)
    if device is None:
        raise RuntimeError('15 秒内未找到 ECG-H2；请确认板子已启动，且未被其他程序连接。')
    return device


def readable_error(error):
    try:
        from bleak.exc import BleakBluetoothNotAvailableError
        if isinstance(error, BleakBluetoothNotAvailableError):
            reason = error.reason.name
            if reason == 'POWERED_OFF':
                return '电脑蓝牙未开启。请在 Windows 设置 → 蓝牙和设备中打开蓝牙，再点击“连接 / 重试”。'
            return f'电脑蓝牙不可用（{reason}）；请检查蓝牙适配器、驱动和系统权限。'
    except ImportError:
        pass
    if isinstance(error, ImportError):
        return f'缺少依赖：{error}。请安装 bleak numpy scipy pyqtgraph PyQt5。'
    return f'{type(error).__name__}: {error}'


class BleThread(threading.Thread):
    def __init__(self, sink):
        super().__init__(name='ECG-BLE', daemon=True)
        self.sink = sink
        self.stop_request = threading.Event()
        self.loop = self.task = self.client = self.command_lock = None
        self.pending_commands = {}
        self.request_id = 0
        self.command_timeout = 10.0

    def stop(self):
        # Disconnect the PC only; it is NOT an ADS STOP command.
        self.stop_request.set()
        loop, task = self.loop, self.task
        if loop is not None and task is not None:
            try:
                loop.call_soon_threadsafe(task.cancel)
            except RuntimeError:
                pass

    def request_command(self, opcode, registers=None):
        values = ctl.normalize_registers(registers or {})
        ctl.encode_command(opcode, 1, values)
        with self.sink.lock:
            if self.client is None or self.loop is None or self.loop.is_closed() or not self.sink.control_supported:
                self.sink.set_control_message('无法发送：尚未连接支持远程控制的新固件。')
                return None
            if self.sink.control_busy:
                self.sink.set_control_message('已有命令正在执行，请等待硬件确认。')
                return None
            self.sink.control_busy = True
            self.sink.control_message = f'{ctl.OP_NAMES[opcode]}：准备发送…'
        coroutine = self.send_command(opcode, values)
        try:
            future = asyncio.run_coroutine_threadsafe(coroutine, self.loop)
        except RuntimeError:
            coroutine.close()
            self.sink.set_control_message('无法发送：蓝牙任务已退出。', busy=False)
            return None
        # Consume exceptions: the UI reads the locked result message, not Qt
        # widgets from this worker callback. CLI can use the same API.
        def consume(done):
            if not done.cancelled():
                done.exception()
        future.add_done_callback(consume)
        return future

    async def send_command(self, opcode, registers=None):
        values = ctl.normalize_registers(registers or {})
        async with self.command_lock:
            self.request_id = self.request_id % 65535 + 1
            request_id = self.request_id
            wire = ctl.encode_command(opcode, request_id, values)
            future = asyncio.get_running_loop().create_future()
            self.pending_commands[request_id] = (opcode, future)
            self.sink.command_sent(opcode, request_id, values)
            try:
                async with asyncio.timeout(self.command_timeout):
                    await self.client.write_gatt_char(ctl.CONTROL_UUID, wire, response=True)
                    reply = await future
                if reply['result'] == ctl.HW_ERROR:
                    raise RuntimeError('ADS 复位 / 配置 / 回读校验失败；设备已停止，GPIO13 低。')
                suffix = '已暂存；ADS 保持停止，开始传输时应用。' if reply['result'] == ctl.STAGED else ctl.STATE_NAMES[reply['state']]
                self.sink.set_control_message(f'{ctl.OP_NAMES[opcode]}：设备确认 · {suffix}', busy=False)
                return reply
            except TimeoutError:
                if opcode != ctl.QUERY:
                    with self.sink.lock:
                        self.sink.control_known = self.sink.stream_config_known = False
                        self.sink.device_state = None
                        self.sink.ecg.lsb_uv = self.sink.resp.lsb_uv = math.nan
                        self.sink._break()
                        self.sink.r_tracker.restart()
                        self.sink.b_tracker.restart()
                self.sink.set_control_message('命令确认超时；可能已执行，不自动重发。请读取设备配置确认。', busy=False)
                raise
            except asyncio.CancelledError:
                self.sink.set_control_message('命令等待已取消；请重连后读取设备状态。', busy=False)
                raise
            except Exception as error:
                self.sink.set_control_message(f'命令失败：{error}', busy=False)
                raise
            finally:
                self.pending_commands.pop(request_id, None)
                with self.sink.lock:
                    self.sink.control_busy = False

    def on_notification(self, _sender, data):
        for reply in self.sink.notification(data) or []:
            pending = self.pending_commands.get(reply['request_id'])
            if pending is not None and pending[0] == reply['opcode'] and not pending[1].done():
                pending[1].set_result(reply)

    async def session(self):
        from bleak import BleakClient
        self.loop = asyncio.get_running_loop()
        self.task = asyncio.current_task()
        self.command_lock = asyncio.Lock()
        if self.stop_request.is_set():
            self.sink.set_connection('已停止')
            return
        self.sink.set_connection('扫描 ECG-H2…')
        device = await find_device()
        disconnected = asyncio.Event()
        loop = self.loop

        def on_disconnect(_):
            if not loop.is_closed():
                loop.call_soon_threadsafe(disconnected.set)

        self.sink.set_connection(f'正在连接 {device.address}…')
        async with BleakClient(device, disconnected_callback=on_disconnect) as client:
            self.client = client
            self.sink.set_connection(f'已连接 {DEVICE_NAME}', connected=True)
            supported = client.services.get_characteristic(ctl.CONTROL_UUID) is not None
            self.sink.set_control_support(supported)
            try:
                await client.start_notify(CHAR_UUID, self.on_notification)
                if supported:
                    try:
                        await self.send_command(ctl.QUERY)
                    except Exception:
                        pass  # retain raw recording; do not pretend query succeeded
                # Reconnection never sends START: a paused board stays paused.
                await disconnected.wait()
            finally:
                self.client = None
                for _opcode, future in self.pending_commands.values():
                    if not future.done(): future.cancel()
        self.sink.set_connection('蓝牙连接已断开；点击“连接 / 重试”开始新会话。')

    def run(self):
        try:
            asyncio.run(self.session())
        except asyncio.CancelledError:
            self.sink.set_connection('已停止')
        except Exception as error:
            message = readable_error(error)
            self.sink.set_connection(message, error=message)
        finally:
            self.client = None
            self.sink.close()


def cli_line(snapshot):
    m = snapshot['metrics']
    battery = number(snapshot['battery_mv'] / 1000 if snapshot['battery_mv'] is not None else None, 3, ' V')
    usage = ' / '.join(number(snapshot['usage'][ch][0] if snapshot['usage'][ch] else None, 1, '%') for ch in ('ch1', 'ch2'))
    leads = ', '.join(name + ':' + snapshot['electrodes'][name][0] for name in ('IN2P', 'IN2N'))
    state = ctl.STATE_NAMES.get(snapshot['device']['state'], '旧固件 / 状态未知')
    imu = snapshot.get('imu')
    if imu and snapshot.get('imu_fresh'):
        imu_text = (f'acc=[{imu["acc_g"][0]:+.2f},{imu["acc_g"][1]:+.2f},{imu["acc_g"][2]:+.2f}]g '
                    f'gyr=[{imu["gyr_dps"][0]:+.1f},{imu["gyr_dps"][1]:+.1f},{imu["gyr_dps"][2]:+.1f}]dps '
                    f'temp={number(imu["temp_c"], 2, " C") if imu["temp_valid"] else "--"}')
    else:
        imu_text = 'BMI270 --'
    return (f"ADS {state}  HR {number(m.get('hr_bpm'), 0, ' bpm')}  "
            f"RR {number(m.get('rr_ms'), 0, ' ms')}  "
            f"呼吸 {number(m.get('resp_bpm'), 1, ' /min')}  电池 {battery}  "
            f"SDNN {number(m.get('sdnn_ms'), 1)} / RMSSD {number(m.get('rmssd_ms'), 1)} ms  "
            f"pNN50 {number(m.get('pnn50_pct'), 1, '%')}  "
            f"ADC CH1/2 {usage}  {imu_text}  丢块 {snapshot['lost_blocks']}  CRC {snapshot['crc_errors']}  {leads}")


def plot_points(samples, origin, scale, sweep=False, filter_revision=None):
    """Insert NaNs at discontinuities; never interpolate missing samples."""
    x, y = [], []
    previous = None
    for sample in samples:
        seconds = (sample.index - origin) / ECG_SPS
        xx = seconds % 5 if sweep else seconds
        value = sample.processed if scale[1] else sample.raw
        if scale[1] and filter_revision is not None and sample.filter_revision != filter_revision:
            value = math.nan
        if previous is not None:
            last, last_x = previous
            if sample.segment != last.segment or xx < last_x:
                x.append(xx)
                y.append(math.nan)
        x.append(xx)
        y.append(value * (sample.lsb_uv if sample.lsb_uv is not None else scale[0]))
        previous = sample, xx
    return x, y


def create_window(start=True, worker_factory=BleThread):
    import pyqtgraph as pg
    from PyQt5 import QtCore, QtWidgets
    pg.setConfigOptions(antialias=False, useOpenGL=False)

    class Window(QtWidgets.QWidget):
        def __init__(self):
            super().__init__()
            self.sink = Sink()
            self.worker = None
            self.filter_dialog = None
            self.register_dialog = None
            self.register_controls = {}
            self.register_refresh_pending = False
            self.filter_controls = {}
            self.loading_filters = False
            self.setWindowTitle('ECG-H2 · 心电 / 呼吸 / 电池监测')
            self.resize(1400, 850)
            self.setStyleSheet('''
                QWidget { background:#101827; color:#dbe5f1; font-family:"Microsoft YaHei UI"; font-size:13px; }
                QPushButton { background:#233650; padding:8px 16px; border:1px solid #36516c; border-radius:5px; }
                QPushButton:hover { background:#315171; } QPushButton:disabled { color:#65758b; }
                QGroupBox { border:1px solid #2c3e54; border-radius:8px; margin-top:12px; padding:12px; }
                QGroupBox::title { subcontrol-origin:margin; left:12px; padding:0 5px; color:#92aecf; }
                QScrollArea { border:0; } QCheckBox { padding:6px; }
            ''')
            root = QtWidgets.QVBoxLayout(self)
            header = QtWidgets.QHBoxLayout()
            title = QtWidgets.QLabel('ECG-H2  /  心电与呼吸')
            title.setStyleSheet('font-size:21px; font-weight:600;')
            header.addWidget(title)
            header.addStretch()
            self.processed_check = QtWidgets.QCheckBox('显示处理后波形')
            self.processed_check.setChecked(True)
            self.processed_check.setToolTip('图形和检测使用当前启用的滤波链；取消勾选仅查看原始波形，不改变检测。')
            header.addWidget(self.processed_check)
            self.filter_button = QtWidgets.QPushButton('滤波设置…')
            self.filter_button.clicked.connect(self.open_filter_settings)
            header.addWidget(self.filter_button)
            self.connect_button = QtWidgets.QPushButton('连接 / 重试')
            self.stop_button = QtWidgets.QPushButton('断开连接')
            header.addWidget(self.connect_button)
            header.addWidget(self.stop_button)
            root.addLayout(header)
            remote = QtWidgets.QHBoxLayout()
            remote.addWidget(QtWidgets.QLabel('ADS 远程控制'))
            self.register_button = QtWidgets.QPushButton('寄存器设置…')
            self.device_start_button = QtWidgets.QPushButton('开始传输')
            self.device_stop_button = QtWidgets.QPushButton('停止传输')
            self.register_button.clicked.connect(self.open_register_settings)
            self.device_start_button.clicked.connect(lambda: self.send_device_command(ctl.START))
            self.device_stop_button.clicked.connect(lambda: self.send_device_command(ctl.STOP))
            for button in (self.register_button, self.device_start_button, self.device_stop_button):
                remote.addWidget(button)
            self.control_label = QtWidgets.QLabel()
            self.control_label.setWordWrap(True)
            remote.addWidget(self.control_label, 1)
            root.addLayout(remote)
            content = QtWidgets.QHBoxLayout()
            root.addLayout(content, 1)
            plots = pg.GraphicsLayoutWidget()
            plots.setBackground('#101827')
            content.addWidget(plots, 3)
            self.resp_plot = plots.addPlot(title='CH1 呼吸 · 10 秒滚动 · 50 SPS')
            self.resp_plot.setLabel('left', 'CH1', units='µV')
            self.resp_plot.setLabel('bottom', '采集时间', units='s')
            self.resp_curve = self.resp_plot.plot(pen=pg.mkPen('#f5a342', width=2))
            plots.nextRow()
            self.ecg_plot = plots.addPlot(title='CH2 ECG · 5 秒扫描 · 500 SPS')
            self.ecg_plot.setLabel('left', 'CH2', units='µV')
            self.ecg_plot.setLabel('bottom', '扫描时间', units='s')
            self.ecg_curve = self.ecg_plot.plot(pen=pg.mkPen('#38bdf8', width=1.5))
            self.r_dots = self.ecg_plot.plot(pen=None, symbol='o', symbolSize=7,
                                           symbolBrush='#fb7185', symbolPen=None)
            self.scan_line = pg.InfiniteLine(angle=90, pen=pg.mkPen('#91a4bb', style=QtCore.Qt.DotLine))
            self.ecg_plot.addItem(self.scan_line)
            self.ecg_plot.setXRange(0, 5, padding=0)
            for plot in (self.resp_plot, self.ecg_plot):
                plot.showGrid(x=True, y=True, alpha=0.18)
                plot.setMouseEnabled(x=False, y=True)
                plot.hideButtons()
                plot.getAxis('left').enableAutoSIPrefix(False)
            sidebar = QtWidgets.QWidget()
            sidebar.setMinimumWidth(350)
            side_layout = QtWidgets.QVBoxLayout(sidebar)
            side_layout.setContentsMargins(3, 0, 3, 0)
            self.metric_label = self.card(side_layout, '实时指标')
            self.hrv_label = self.card(side_layout, 'HRV · 最近 60 秒 · 时域')
            self.imu_label = self.card(side_layout, 'BMI270 / IMU / Temperature')
            self.status_label = self.card(side_layout, '设备 / 导联 / ADC 状态')
            side_layout.addStretch()
            scroll = QtWidgets.QScrollArea()
            scroll.setWidgetResizable(True)
            scroll.setMinimumWidth(390)
            scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
            scroll.setWidget(sidebar)
            content.addWidget(scroll, 1)
            self.connection_label = QtWidgets.QLabel()
            self.connection_label.setWordWrap(True)
            root.addWidget(self.connection_label)
            self.file_label = QtWidgets.QLabel('收到第一帧有效数据后开始保存到 date/；每次会话一个 CSV。')
            self.file_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
            self.file_label.setWordWrap(True)
            root.addWidget(self.file_label)
            caption = QtWidgets.QLabel('滤波器独立开关，默认仅高通；原始数据不改写。配置变更后重新预热，检测不跨配置拼接。红点 / 短窗指标仅供实验，不用于诊断。')
            caption.setStyleSheet('color:#8095af; font-size:11px;')
            caption.setWordWrap(True)
            root.addWidget(caption)
            self.connect_button.clicked.connect(self.connect_device)
            self.stop_button.clicked.connect(self.stop_device)
            self.timer = QtCore.QTimer(self)
            self.timer.timeout.connect(self.update_display)
            self.timer.start(100)
            self.update_display()
            if start:
                QtCore.QTimer.singleShot(0, self.connect_device)

        def card(self, layout, title):
            box = QtWidgets.QGroupBox(title)
            group = QtWidgets.QVBoxLayout(box)
            label = QtWidgets.QLabel()
            label.setTextFormat(QtCore.Qt.RichText)
            label.setWordWrap(True)
            label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
            group.addWidget(label)
            layout.addWidget(box)
            return label

        def connect_device(self):
            if self.worker is not None and self.worker.is_alive():
                return
            # A retry/reconnect is a new acquisition session, never a new file
            # inside an already running session.
            selected = self.sink.get_filter_settings()
            self.sink.close()
            self.sink = Sink(filter_settings=selected)
            self.worker = worker_factory(self.sink)
            self.worker.start()
            self.update_display()

        def send_device_command(self, opcode, registers=None):
            try:
                if self.worker is None:
                    self.sink.set_control_message('无法发送：请先连接设备。')
                    return None
                future = self.worker.request_command(opcode, registers)
                if future is not None and opcode in (ctl.QUERY, ctl.SET_REGISTERS):
                    self.register_refresh_pending = True
                self.update_display()
                return future
            except (ValueError, RuntimeError) as error:
                self.sink.set_control_message(f'命令未发送：{error}')
                self.update_display()
                return None

        def load_register_controls(self):
            registers = self.sink.snapshot(False)['device']['registers']
            for register in ctl.REGISTERS:
                self.register_controls[register.address].setValue(registers[register.name])

        def apply_register_controls(self):
            values = {}
            for address, spin in self.register_controls.items():
                spin.interpretText()
                values[address] = spin.value()
            self.send_device_command(ctl.SET_REGISTERS, values)

        def open_register_settings(self):
            if self.register_dialog is None:
                self.register_dialog = QtWidgets.QDialog(self)
                self.register_dialog.setWindowTitle('ADS1292R · 白名单寄存器配置')
                self.register_dialog.setMinimumWidth(740)
                layout = QtWidgets.QVBoxLayout(self.register_dialog)
                explanation = QtWidgets.QLabel(
                    '仅允许以下 7 个寄存器；十六进制字节值，保留位和编码会校验。\n'
                    '采集中应用：暂停 → 硬件复位 → 配置 / 回读校验 → 恢复。\n'
                    '停止时应用：仅暂存，ADS 保持未配置，直到“开始传输”。设置仅保存在 RAM，上电恢复固件默认值。')
                explanation.setWordWrap(True)
                layout.addWidget(explanation)
                grid = QtWidgets.QGridLayout()
                for column, text in enumerate(('寄存器', '地址', '设置值', '二进制', '字段提示')):
                    grid.addWidget(QtWidgets.QLabel(text), 0, column)
                for row, register in enumerate(ctl.REGISTERS, 1):
                    spin = QtWidgets.QSpinBox()
                    spin.setRange(0, 255)
                    spin.setDisplayIntegerBase(16)
                    spin.setPrefix('0x')
                    spin.setKeyboardTracking(False)
                    spin.setMinimumWidth(85)
                    spin.setToolTip(register.hint)
                    binary = QtWidgets.QLabel()
                    binary.setMinimumWidth(80)
                    spin.valueChanged.connect(lambda value, label=binary: label.setText(f'{value:08b}'))
                    hint = QtWidgets.QLabel(register.hint)
                    hint.setWordWrap(True)
                    self.register_controls[register.address] = spin
                    for col, widget in enumerate((QtWidgets.QLabel(register.name),
                                                  QtWidgets.QLabel(f'0x{register.address:02X}'), spin, binary, hint)):
                        grid.addWidget(widget, row, col)
                    spin.setValue(register.default)
                    binary.setText(f'{register.default:08b}')
                layout.addLayout(grid)
                self.register_result_label = QtWidgets.QLabel()
                self.register_result_label.setWordWrap(True)
                layout.addWidget(self.register_result_label)
                buttons = QtWidgets.QHBoxLayout()
                self.register_query_button = QtWidgets.QPushButton('读取设备配置')
                self.register_query_button.setToolTip('查询固件缓存：运行时为上次通过回读校验的配置；停止时为待应用配置，不触发硬件复位。')
                self.register_query_button.clicked.connect(lambda: self.send_device_command(ctl.QUERY))
                defaults = QtWidgets.QPushButton('填入默认值')
                defaults.setToolTip('只修改编辑框；需要点击“应用到设备”才发送。')
                defaults.clicked.connect(lambda: [self.register_controls[r.address].setValue(r.default) for r in ctl.REGISTERS])
                self.register_apply_button = QtWidgets.QPushButton('应用到设备')
                self.register_apply_button.clicked.connect(self.apply_register_controls)
                close = QtWidgets.QPushButton('关闭')
                close.clicked.connect(self.register_dialog.hide)
                for button in (self.register_query_button, defaults, self.register_apply_button, close):
                    buttons.addWidget(button)
                layout.addLayout(buttons)
            self.load_register_controls()
            self.update_display()
            self.register_dialog.show()
            self.register_dialog.raise_()
            self.register_dialog.activateWindow()

        def open_filter_settings(self):
            if self.filter_dialog is None:
                self.filter_dialog = QtWidgets.QDialog(self)
                self.filter_dialog.setWindowTitle('滤波设置 · 每路独立启用')
                self.filter_dialog.setMinimumWidth(580)
                layout = QtWidgets.QVBoxLayout(self.filter_dialog)
                explanation = QtWidgets.QLabel(
                    '开关和参数即时应用：高通 → ECG 陷波 → 低通。关闭所有项即旁路。\n'
                    '改变配置会重置该通道的检测窗口和处理状态，先重新预热；原始数据始终保存。')
                explanation.setWordWrap(True)
                layout.addWidget(explanation)
                for name, title, fs in (('ecg', 'CH2 ECG · 500 SPS', ECG_SPS),
                                       ('resp', 'CH1 呼吸 · 50 SPS', RESP_SPS)):
                    box = QtWidgets.QGroupBox(title)
                    grid = QtWidgets.QGridLayout(box)
                    grid.addWidget(QtWidgets.QLabel('启用 / 类型'), 0, 0)
                    grid.addWidget(QtWidgets.QLabel('频率 (Hz)'), 0, 1)
                    if name == 'ecg':
                        grid.addWidget(QtWidgets.QLabel('Q'), 0, 2)
                    controls = {}
                    stages = [('hp', '高通 · 二阶')]
                    if name == 'ecg':
                        stages.append(('notch', '工频陷波 · 二阶'))
                    stages.append(('lp', '低通 · 二阶'))
                    for row, (key, label) in enumerate(stages, 1):
                        check = QtWidgets.QCheckBox(label)
                        frequency = QtWidgets.QDoubleSpinBox()
                        frequency.setDecimals(3)
                        frequency.setRange(0.001, fs / 2 - 0.001)
                        frequency.setSingleStep(0.01 if key == 'hp' else 1.0 if name == 'ecg' else 0.1)
                        frequency.setKeyboardTracking(False)
                        frequency.setToolTip(f'必须在 0 与 {fs / 2:g} Hz 之间；高通 / 低通为截止频率。')
                        controls[key + '_enabled'] = check
                        controls[key + '_hz'] = frequency
                        grid.addWidget(check, row, 0)
                        grid.addWidget(frequency, row, 1)
                        if key == 'notch':
                            quality = QtWidgets.QDoubleSpinBox()
                            quality.setRange(1.0, 200.0)
                            quality.setDecimals(1)
                            quality.setSingleStep(1.0)
                            quality.setKeyboardTracking(False)
                            quality.setToolTip('Q 越高，陷波越窄；带宽约为中心频率 / Q。')
                            controls['notch_q'] = quality
                            grid.addWidget(quality, row, 2)
                    self.filter_controls[name] = controls
                    layout.addWidget(box)
                self.filter_error = QtWidgets.QLabel()
                self.filter_error.setWordWrap(True)
                self.filter_error.setStyleSheet('color:#fbbf24;')
                layout.addWidget(self.filter_error)
                hint = QtWidgets.QLabel('默认仅高通；低通和陷波默认关闭。呼吸为 50 SPS，不提供 50 Hz 工频陷波。')
                hint.setWordWrap(True)
                layout.addWidget(hint)
                buttons = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Close)
                buttons.button(QtWidgets.QDialogButtonBox.Close).setText('关闭')
                reset = buttons.addButton('恢复默认', QtWidgets.QDialogButtonBox.ResetRole)
                reset.clicked.connect(self.reset_filter_settings)
                buttons.rejected.connect(self.filter_dialog.hide)
                layout.addWidget(buttons)
                for controls in self.filter_controls.values():
                    for widget in controls.values():
                        signal = widget.toggled if isinstance(widget, QtWidgets.QCheckBox) else widget.valueChanged
                        signal.connect(lambda _value: self.apply_filter_controls())
            self.load_filter_controls(self.sink.get_filter_settings())
            self.filter_dialog.show()
            self.filter_dialog.raise_()
            self.filter_dialog.activateWindow()

        def load_filter_controls(self, settings):
            self.loading_filters = True
            try:
                for name, controls in self.filter_controls.items():
                    values = settings[name].to_dict()
                    for key, widget in controls.items():
                        if isinstance(widget, QtWidgets.QCheckBox):
                            widget.setChecked(values[key])
                        else:
                            widget.setValue(values[key])
                self.filter_error.clear()
                self.set_filter_control_enabled()
            finally:
                self.loading_filters = False

        def set_filter_control_enabled(self):
            for controls in self.filter_controls.values():
                for stage in ('hp', 'lp', 'notch'):
                    check = controls.get(stage + '_enabled')
                    if check is not None:
                        controls[stage + '_hz'].setEnabled(check.isChecked())
                        if stage == 'notch':
                            controls['notch_q'].setEnabled(check.isChecked())

        def apply_filter_controls(self):
            if self.loading_filters:
                return
            self.set_filter_control_enabled()
            selected = {}
            for name, controls in self.filter_controls.items():
                values = FilterSettings.defaults(name).to_dict()
                values.update({key: widget.isChecked() if isinstance(widget, QtWidgets.QCheckBox) else widget.value()
                               for key, widget in controls.items()})
                selected[name] = FilterSettings(**values)
            try:
                changes = self.sink.configure_filters(**selected)
            except ValueError as error:
                self.filter_error.setText(f'未应用：{error}。当前运行配置不变；修正参数后会自动应用。')
                return
            self.filter_error.setText('已应用；更改的通道正在重新预热。' if changes else '')
            self.update_display()

        def reset_filter_settings(self):
            self.load_filter_controls({name: FilterSettings.defaults(name) for name in ('ecg', 'resp')})
            self.apply_filter_controls()

        def stop_device(self):
            if self.worker is not None and self.worker.is_alive():
                self.worker.stop()
                self.connection_label.setText('正在断开 BLE 并关闭会话文件；不会发送 ADS 停止命令…')
                self.stop_button.setEnabled(False)

        def closeEvent(self, event):
            self.timer.stop()
            if self.worker is not None and self.worker.is_alive():
                self.worker.stop()
                self.worker.join(timeout=5)
                if self.worker.is_alive():
                    # Do not close a file from under a still-running BLE worker.
                    self.connection_label.setText('等待蓝牙任务退出… 请稍后再次关闭窗口。')
                    self.timer.start(100)
                    event.ignore()
                    return
            self.sink.close()
            if self.filter_dialog is not None:
                self.filter_dialog.close()
            if self.register_dialog is not None:
                self.register_dialog.close()
            event.accept()

        def update_display(self):
            snap = self.sink.snapshot()
            m = snap['metrics']
            can_remote = snap['connected'] and snap['control_supported'] and not snap['control_busy']
            for button in (self.register_button, self.device_start_button, self.device_stop_button):
                button.setEnabled(can_remote)
            self.control_label.setText(snap['control_message'])
            self.control_label.setStyleSheet('color:#fbbf24;' if any(w in snap['control_message'] for w in ('失败', '超时', '未发送')) else 'color:#92aecf;')
            if self.register_dialog is not None:
                self.register_query_button.setEnabled(can_remote)
                self.register_apply_button.setEnabled(can_remote and snap['device']['known'])
                self.register_result_label.setText(snap['control_message'])
                if self.register_refresh_pending and not snap['control_busy']:
                    self.register_refresh_pending = False
                    if snap['device']['known']:
                        self.load_register_controls()
            processed = self.processed_check.isChecked()
            ecg, resp = snap['ecg'], snap['resp']
            if resp:
                newest = resp[-1].index
                tail = [s for s in resp if s.index >= newest - 10 * ECG_SPS]
                x, y = plot_points(tail, snap['origin'], (LSB_CH1_UV, processed),
                                   filter_revision=snap['filters']['resp']['revision'])
                self.resp_curve.setData(x, y, connect='finite')
                end = (newest - snap['origin']) / ECG_SPS
                self.resp_plot.setXRange(max(0, end - 10), max(10, end), padding=0)
                self.fit_y(self.resp_plot, y)
            else:
                self.resp_curve.setData([], [])
                self.resp_plot.setXRange(0, 10, padding=0)
            if ecg:
                newest = ecg[-1].index
                tail = [s for s in ecg if s.index >= newest - 5 * ECG_SPS]
                x, y = plot_points(tail, snap['origin'], (LSB_CH2_UV, processed), sweep=True,
                                   filter_revision=snap['filters']['ecg']['revision'])
                self.ecg_curve.setData(x, y, connect='finite')
                self.fit_y(self.ecg_plot, y)
                peaks = [s for s in snap['r_peaks'] if s.index >= newest - 5 * ECG_SPS]
                self.r_dots.setData([(s.index - snap['origin']) / ECG_SPS % 5 for s in peaks],
                                    [(s.processed if processed else s.raw) * (s.lsb_uv if s.lsb_uv is not None else LSB_CH2_UV) for s in peaks])
                self.scan_line.setPos((newest - snap['origin']) / ECG_SPS % 5)
            else:
                self.ecg_curve.setData([], [])
                self.r_dots.setData([], [])
            ecg_source = FilterSettings(**snap['filters']['ecg']['settings']).description() if processed else '原始值'
            resp_source = FilterSettings(**snap['filters']['resp']['settings']).description() if processed else '原始值'
            self.resp_plot.setTitle(f'CH1 呼吸 · 10 秒滚动 · 50 SPS · {resp_source}')
            self.ecg_plot.setTitle(f'CH2 ECG · 5 秒扫描 · 500 SPS · {ecg_source}')
            battery = number(snap['battery_mv'] / 1000 if snap['battery_mv'] is not None else None, 3)
            self.metric_label.setText(
                '<table width="100%"><tr><td>心率</td><td>呼吸频率</td><td>电池电压</td></tr>'
                f'<tr><td><span style="font-size:28px;color:#38bdf8">{number(m.get("hr_bpm"))}</span></td>'
                f'<td><span style="font-size:28px;color:#f5a342">{number(m.get("resp_bpm"), 1)}</span></td>'
                f'<td><span style="font-size:24px;color:#34d399">{battery}</span></td></tr>'
                '<tr><td>bpm</td><td>次 / 分</td><td>V · 每 5 s</td></tr></table><br>'
                f'平均心率　{number(m.get("avg_hr_bpm"), 0, " bpm")}<br>'
                f'最近 RR　 {number(m.get("rr_ms"), 0, " ms")}<br>'
                f'呼吸周期　{number(m.get("resp_period_s"), 2, " s")}<br>'
                f'<span style="color:#8292a7">累计 R 候选 {snap["r_total"]} / 呼吸峰 {snap["breath_total"]}</span>')
            self.hrv_label.setText(
                f'SDNN　　 {number(m.get("sdnn_ms"), 1, " ms")}<br>'
                f'RMSSD　　{number(m.get("rmssd_ms"), 1, " ms")}<br>'
                f'pNN50　　 {number(m.get("pnn50_pct"), 1, " %")}<br>'
                f'有效 NN {m.get("nn_valid", 0)} / 排除 RR {m.get("nn_rejected", 0)}<br>'
                '<span style="color:#8292a7;font-size:11px">仅相邻且连续的有效 NN 参与差分；短窗结果仅作实验参考。</span>')
            imu = snap['imu']
            if imu and snap['imu_fresh']:
                ax, ay, az = imu['acc_g']
                gx, gy, gz = imu['gyr_dps']
                temp_text = number(imu['temp_c'], 2, ' C') if imu['temp_valid'] else '--'
                self.imu_label.setText(
                    f'accel [{ax:+.3f}, {ay:+.3f}, {az:+.3f}] g<br>'
                    f'gyro [{gx:+.2f}, {gy:+.2f}, {gz:+.2f}] dps<br>'
                    f'temperature {temp_text}<br>'
                    f'{imu["source"]} / status 0x{imu["status"]:02X} / '
                    f'sample {imu["sequence"]} / total {snap["imu_samples"]}')
            else:
                self.imu_label.setText('BMI270 data waiting / stale')
            status = snap['status'] or {}
            connection_color = '#34d399' if snap['connected'] else '#8292a7'
            stream_color = '#34d399' if snap['data_fresh'] else '#fbbf24'
            lines = [f'BLE <span style="color:{connection_color}">{"已连接" if snap["connected"] else "未连接"}</span>'
                     f'　数据 <span style="color:{stream_color}">{"实时" if snap["data_fresh"] else "等待 / 中断"}</span>']
            imu_color = '#34d399' if snap['imu_fresh'] else '#fbbf24'
            lines.append(f'BMI270 <span style="color:{imu_color}">{"live" if snap["imu_fresh"] else "waiting / stale"}</span>')
            state = ctl.STATE_NAMES.get(snap['device']['state'], '尚未读取' if snap['control_supported'] else '固定默认配置 / 无远程控制')
            lines.append(f'<b>ADS {state}</b>')
            if snap['device']['known'] or not snap['control_supported']:
                lines.append(f'CH1 增益 ×{snap["device"]["ch1_gain"]} / CH2 增益 ×{snap["device"]["ch2_gain"]}')
            if snap['device']['known']:
                regs = snap['device']['registers']
                if regs['CH2SET'] & 0x8F:
                    lines.append('<span style="color:#fbbf24">CH2 已关闭 / 非电极输入，暂停 R 波检测。</span>')
                if regs['CH1SET'] & 0x8F or regs['RESP1'] & 0xC0 != 0xC0:
                    lines.append('<span style="color:#fbbf24">CH1 输入 / 呼吸解调未启用，暂停呼吸检测。</span>')
            lines.append('状态包：' + ('有效' if snap['status_fresh'] else '尚无 / 超过 8 秒未更新'))
            for name in ('IN2P', 'IN2N', 'IN1P', 'IN1N', 'RLD'):
                text, color = snap['electrodes'][name]
                lines.append(f'{name}　<span style="color:{color}">{text}</span>')
            lines.append('<b>ADC 原始幅度 / 距离饱和</b>')
            for name, label in (('ch1', 'CH1 呼吸'), ('ch2', 'CH2 ECG')):
                usage = snap['usage'][name]
                value = f'使用 {usage[0]:.2f}% / 余量 {usage[1]:.2f}%' if usage else '--'
                color = '#fb7185' if usage and usage[0] >= 98 else '#dbe5f1'
                lines.append(f'{label}　<span style="color:{color}">{value}</span>')
            lines.extend([
                '<span style="color:#8292a7;font-size:11px">近 5 秒原始峰值 + 最新固件窗口（含 DC）。CH1 是块均值，不能排除块内瞬时饱和。</span>',
                f'固件窗口 CH2 近饱和　{status.get("saturation_count", "--")} 次',
                f'DRDY 丢样 {status.get("dropped_total", "--")} / 坏帧 {status.get("bad_frames", "--")}',
                f'传输丢块 {snap["lost_blocks"]} / CRC 错误 {snap["crc_errors"]} / 格式错误 {snap["malformed"]}',
                f'采集 {snap["samples"]} ECG 点 / {snap["blocks"]} 呼吸点',
            ])
            lines.append(f'BMI270 IMU samples {snap["imu_samples"]}')
            if snap['last_flags'] & 0x0F:
                lines.append('<span style="color:#fbbf24">该包有导联脱落 / 补样，已排除相关检测。</span>')
            if status.get('reason') == 2:
                lines.append('<span style="color:#fb7185">DRDY 中断：检查 ADS 采样与接线。</span>')
            self.status_label.setText('<br>'.join(lines))
            alive = self.worker is not None and self.worker.is_alive()
            self.connect_button.setEnabled(not alive)
            self.stop_button.setEnabled(alive and not self.worker.stop_request.is_set())
            self.connection_label.setText(snap['connection'])
            self.connection_label.setStyleSheet('color:#fb7185;' if snap['error'] else 'color:#92aecf;')
            if snap['path']:
                state = '会话文件（已关闭）' if self.sink.closed else '正在保存'
                self.file_label.setText(f'{state}：{snap["path"]}')
            else:
                self.file_label.setText(f'等待有效数据后保存到：{SAVE_DIR}（一个会话一个 CSV）')

        @staticmethod
        def fit_y(plot, values):
            finite = [v for v in values if math.isfinite(v)]
            if finite:
                lo, hi = min(finite), max(finite)
                padding = max(1.0, (hi - lo) * 0.12)
                plot.setYRange(lo - padding, hi + padding, padding=0)

    return Window()


def cli_command_reader(worker):
    while not worker.stop_request.is_set():
        line = sys.stdin.readline()
        if not line:
            return
        try:
            command = ctl.parse_cli_command(line)
            if command is not None:
                worker.request_command(*command)
        except (ValueError, RuntimeError) as error:
            print(f'命令未发送：{error}', flush=True)


def run_cli():
    sink = Sink()
    worker = BleThread(sink)
    worker.start()
    print('扫描 ECG-H2…  Ctrl+C 停止。默认仅高通；原始和处理后数据保存至 date/。', flush=True)
    if sys.stdin.isatty():
        print('设备命令：query / start / stop / set RESP1=0xF2 CH1SET=0x40 ...；Ctrl+C 仅断开电脑连接。', flush=True)
        threading.Thread(target=cli_command_reader, args=(worker,), name='ECG-CLI-commands', daemon=True).start()
    previous = None
    previous_control = None
    last_summary = 0.0
    try:
        while worker.is_alive():
            snap = sink.snapshot(include_samples=False)
            if snap['connection'] != previous:
                previous = snap['connection']
                print(previous, flush=True)
            if snap['control_message'] != previous_control:
                previous_control = snap['control_message']
                print(previous_control, flush=True)
            if snap['samples'] and time.monotonic() - last_summary >= 1:
                print(cli_line(snap), flush=True)
                last_summary = time.monotonic()
            time.sleep(0.1)
    except KeyboardInterrupt:
        print('\n正在停止并保存…', flush=True)
        worker.stop()
    finally:
        worker.join()
    snap = sink.snapshot(include_samples=False)
    if snap['connection'] != previous:
        print(snap['connection'], flush=True)
    if snap['path']:
        print('会话文件：' + snap['path'], flush=True)
    return 1 if snap['error'] else 0


def main(argv=None):
    parser = argparse.ArgumentParser(description='ECG-H2 BLE 心电 / 呼吸 / 电池采集；默认打开本地 UI。')
    parser.add_argument('--cli', action='store_true', help='命令行采集模式（不打开 UI）')
    args = parser.parse_args(argv)
    if args.cli:
        return run_cli()
    try:
        from PyQt5 import QtWidgets
        app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv[:1])
        window = create_window()
        window.show()
        return app.exec_()
    except ImportError as error:
        print(readable_error(error), file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
