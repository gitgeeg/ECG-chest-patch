"""Whitelisted ADS1292R BLE controls; no arbitrary SPI/register access."""
from dataclasses import dataclass
import struct

CONTROL_UUID = '6cec0000-106f-3e9a-214d-7b5ca1e30003'
VERSION = 1
QUERY, START, STOP, SET_REGISTERS = 1, 2, 3, 4
OK, STAGED, HW_ERROR = 0, 1, 2
STOPPED, RUNNING, FAULT = 0, 1, 2
STATE_NAMES = {STOPPED: '已停止（ADS 未配置）', RUNNING: '正在传输', FAULT: '配置失败（ADS 已停止）'}
OP_NAMES = {QUERY: '读取设备配置', START: '开始传输', STOP: '停止传输', SET_REGISTERS: '修改寄存器'}
COMMAND_HEADER = struct.Struct('<BBHB')
REPLY_HEADER = struct.Struct('<BBHBBIIB')
REPLY_SIZE = 29

@dataclass(frozen=True)
class Register:
    name: str
    address: int
    default: int
    hint: str

REGISTERS = (
    Register('LOFF', 3, 0xF9, '比较器阈值 / 脱落检测电流；bit4=1，bit1=0'),
    Register('CH1SET', 4, 0x40, 'CH1 电源 / 增益 / 输入选择；增益编码 000/001/010/011/100/101/110 = 6/1/2/3/4/8/12'),
    Register('CH2SET', 5, 0x00, 'CH2 电源 / 增益 / 输入选择；关闭通道时必须选择输入短路 MUX=1'),
    Register('RLD_SENS', 6, 0xAC, 'RLD 电源 / 检测 / 输入；CHOP=01 为保留编码'),
    Register('LOFF_SENS', 7, 0x00, '脱落检测输入 / 极性；bits7:6=00'),
    Register('RESP1', 9, 0xF2, '呼吸调制 / 解调 / 相位 / 时钟；bit1 必须为 1'),
    Register('RESP2', 10, 0x83, '偏移校准 / 呼吸频率 / RLD 参考；bits6:3=0000，bit0=1'),
)
BY_ADDRESS = {r.address: r for r in REGISTERS}
BY_NAME = {r.name: r for r in REGISTERS}
DEFAULT_REGISTERS = {r.address: r.default for r in REGISTERS}


def validate_register(address, value):
    if address not in BY_ADDRESS:
        raise ValueError(f'不允许修改寄存器地址 {address!r}')
    if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= 255:
        raise ValueError('寄存器值必须是 0x00–0xFF 的整数')
    valid = True
    if address == 3:
        valid = value & 0x12 == 0x10
    elif address in (4, 5):
        valid = (value >> 4) & 7 != 7 and value & 15 <= 9 and (not value & 0x80 or value & 15 == 1)
    elif address == 6:
        valid = value >> 6 != 1
    elif address == 7:
        valid = not value & 0xC0
    elif address == 9:
        valid = bool(value & 2)
    elif address == 10:
        valid = value & 0x79 == 1
    if not valid:
        raise ValueError(f'{BY_ADDRESS[address].name}=0x{value:02X} 无效；{BY_ADDRESS[address].hint}')
    return value


def normalize_registers(registers):
    result = {}
    for key, value in registers.items():
        if isinstance(key, str):
            name = key.upper()
            if name not in BY_NAME:
                raise ValueError(f'寄存器 {key} 不在白名单中')
            address = BY_NAME[name].address
        else:
            address = key
        if isinstance(address, bool) or not isinstance(address, int):
            raise ValueError('寄存器地址必须为整数或白名单中的名称')
        if address in result:
            raise ValueError('同一命令不能重复设置一个寄存器')
        result[address] = validate_register(address, value)
    return result


def encode_command(opcode, request_id, registers=None):
    if opcode not in OP_NAMES or isinstance(opcode, bool):
        raise ValueError('不支持的设备命令')
    if isinstance(request_id, bool) or not isinstance(request_id, int) or not 1 <= request_id <= 65535:
        raise ValueError('请求序号必须是 1–65535')
    values = normalize_registers(registers or {})
    if (opcode == SET_REGISTERS and not 1 <= len(values) <= 7) or (opcode != SET_REGISTERS and values):
        raise ValueError('只有 SET 命令允许携带 1–7 个寄存器')
    out = bytearray(COMMAND_HEADER.pack(VERSION, opcode, request_id, len(values)))
    for address in sorted(values):
        out.extend((address, values[address]))
    return bytes(out)


def decode_reply(payload):
    if len(payload) != REPLY_SIZE:
        raise ValueError('设备控制回复长度错误')
    version, opcode, request_id, result, state, revision, epoch, count = REPLY_HEADER.unpack_from(payload)
    if version != VERSION or opcode not in OP_NAMES or result not in (OK, STAGED, HW_ERROR) or state not in STATE_NAMES or count != 7:
        raise ValueError('设备控制回复字段错误')
    registers = {}
    for offset in range(REPLY_HEADER.size, len(payload), 2):
        address, value = payload[offset:offset+2]
        if address in registers:
            raise ValueError('设备回复寄存器重复')
        registers[address] = validate_register(address, value)
    if set(registers) != set(BY_ADDRESS):
        raise ValueError('设备回复缺少白名单寄存器')
    return {'opcode': opcode, 'request_id': request_id, 'result': result, 'state': state,
            'revision': revision, 'epoch': epoch, 'registers': registers}


def channel_gain(registers, address):
    return (6, 1, 2, 3, 4, 8, 12)[(registers[address] >> 4) & 7]


def parse_cli_command(line):
    parts = line.strip().split()
    if not parts:
        return None
    opcode = {'query': QUERY, 'start': START, 'stop': STOP, 'set': SET_REGISTERS}.get(parts[0].lower())
    if opcode is None:
        raise ValueError('命令：query / start / stop / set RESP1=0xF2 CH1SET=0x40 ...')
    values = {}
    if opcode != SET_REGISTERS and len(parts) != 1:
        raise ValueError('query/start/stop 不接受额外参数')
    for item in parts[1:]:
        if '=' not in item:
            raise ValueError('寄存器参数格式为 NAME=0xNN')
        key, text = item.split('=', 1)
        if key.upper() in values:
            raise ValueError('寄存器名称重复')
        values[key.upper()] = int(text, 0)
    values = normalize_registers(values)
    encode_command(opcode, 1, values)  # perform complete validation before sending
    return opcode, values
