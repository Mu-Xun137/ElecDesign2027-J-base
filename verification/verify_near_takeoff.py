"""Replay actual Cortex-M3 JCore code with bounded lift-off input traces.

Requires unicorn==2.1.4. Six-ms replay steps are inside the 25-ms controller
deadline; real 3-ms scheduling/PWM publication is covered by other checks.
This is not a flight-dynamics model or a proof of available thrust.
"""
from pathlib import Path
import argparse
import hashlib
import json
import struct
import sys

from build import ROOT, hex_memory
from verify_timer import symbol_address


def symbol_size(path, target):
    data = path.read_bytes()
    h = struct.unpack_from('<16sHHIIIIIHHHHHH', data)
    sections = [struct.unpack_from('<10I', data, h[6] + k * h[11]) for k in range(h[12])]
    for section in sections:
        if section[1] != 2:
            continue
        strings = sections[section[6]]
        names = data[strings[4]:strings[4] + strings[5]]
        for at in range(section[4], section[4] + section[5], section[9]):
            n, _, size, _, _, _ = struct.unpack_from('<IIIBBH', data, at)
            if names[n:names.find(b'\0', n)].decode('ascii', errors='replace') == target:
                return size
    raise ValueError('Missing symbol size: ' + target)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--variant', choices=['basic', 'upgrade_single', 'upgrade_ab'], default='basic')
    parser.add_argument('--dependency-path', type=Path)
    args = parser.parse_args()
    if args.dependency_path:
        sys.path.insert(0, str(args.dependency_path.resolve()))
    import unicorn
    from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, arm_const
    axf = ROOT / 'firmware' / args.variant / ('flight_' + args.variant + '.axf')
    hexfile = axf.with_suffix('.hex')
    assert symbol_size(axf, 'j_flight') == 356
    assert symbol_size(axf, 'j_sensor') == 144
    assert symbol_size(axf, 'command') == 116
    memory, _, _ = hex_memory(hexfile)
    image = bytearray(b'\xff' * 0x10000)
    for address, value in memory.items():
        image[address - 0x08000000] = value
    init = symbol_address(axf, 'JCore_Init')
    step = symbol_address(axf, 'JCore_Step')

    class Fixture:
        def __init__(self, now=1000):
            self.cpu = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
            self.cpu.mem_map(0x08000000, 0x10000)
            self.cpu.mem_write(0x08000000, bytes(image))
            self.cpu.mem_map(0x20000000, 0x5000)
            self.cpu.mem_map(0x10000000, 0x1000)
            self.core, self.input, self.stack = 0x20003000, 0x20003200, 0x20004f00
            self.now = now
            self.channels = [1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000]
            self.range = self.velocity = self.flow = False
            self.module = self.imu = True
            self.height, self.vz, self.yaw = 766.0, 0.0, 0.0
            self.range_ms = self.flow_ms = self.module_ms = 0
            self.call(init, self.core, now)
            self.cpu.mem_write(self.core + 16, b'\x05')
            self.cpu.mem_write(self.core + 100, struct.pack('<I', 1))
            self.cpu.mem_write(self.core + 132, struct.pack('<f', 100.0))

        def call(self, entry, *values):
            for name in ['MSP', 'SP']:
                self.cpu.reg_write(getattr(arm_const, 'UC_ARM_REG_' + name), self.stack)
            self.cpu.reg_write(arm_const.UC_ARM_REG_LR, 0x10000001)
            for k, value in enumerate(values):
                self.cpu.reg_write(getattr(arm_const, 'UC_ARM_REG_R' + str(k)), value)
            self.cpu.emu_start(entry | 1, 0x10000000, timeout=1000000, count=100000)
            assert self.cpu.reg_read(arm_const.UC_ARM_REG_PC) == 0x10000000

        def byte(self, offset):
            return self.cpu.mem_read(self.core + offset, 1)[0]

        def floating(self, offset):
            return struct.unpack('<f', self.cpu.mem_read(self.core + offset, 4))[0]

        def word(self, offset):
            return struct.unpack('<I', self.cpu.mem_read(self.core + offset, 4))[0]

        def tick(self, events=0):
            self.now = (self.now + 6) & 0xffffffff
            if self.range:
                self.range_ms = self.now
            if self.flow:
                self.flow_ms = self.now
            if self.module:
                self.module_ms = self.now
            data = bytearray(116)
            struct.pack_into('<II8H4B', data, 0, self.now, self.now, *self.channels, 1, 1, events, 0)
            struct.pack_into('<5I', data, 28, self.range_ms, self.flow_ms, self.now, self.now, 1)
            floats = [self.height, self.vz, 0, 0, 0, 0, 0, 0, 0, 0, 100, 0, 0, self.yaw]
            struct.pack_into('<14f', data, 48, *floats)
            struct.pack_into('<8B', data, 104, int(self.range), int(self.velocity), int(self.flow),
                             int(self.imu), 1, 80, int(self.module), 0)
            struct.pack_into('<I', data, 112, self.module_ms)
            self.cpu.mem_write(self.input, bytes(data))
            self.call(step, self.core, self.input)

        def advance(self, ms):
            for _ in range(0, ms, 6):
                self.tick()

        def arm(self):
            self.tick()
            self.channels[2] = 1819
            self.advance(90)
            self.channels[2] = 1000
            self.tick()
            assert self.byte(5) == 1 and self.byte(6) == 0 and self.byte(336) == 1

        def start(self):
            self.arm()
            self.tick(4)
            assert self.byte(0) == 3 and self.byte(337) == 1 and self.byte(336) == 0

    def bounded_timeout(now=1000):
        f = Fixture(now)
        f.tick(4)
        assert f.byte(5) == 0 and f.floating(160) == 0
        f.start()
        started = f.word(344)
        while f.byte(5):
            before = f.floating(160)
            f.tick()
            if f.byte(5):
                assert f.byte(337) == 1 and f.byte(7) == 0 and f.byte(8) == 0
                assert 0 <= f.floating(160) <= 400
                assert 0 <= f.floating(160) - before <= 2.701
            assert ((f.now - started) & 0xffffffff) <= 2004
        assert f.byte(3) == 9 and f.floating(160) == 0 and f.byte(337) == 0
        f.tick(4)
        assert f.byte(5) == 0 and f.floating(160) == 0

    def emergency_and_io():
        for events, io, fault in [(2 | 4 | 8, True, 1), (0, False, 5)]:
            f = Fixture()
            f.start()
            f.advance(300)
            f.module = io
            f.tick(events)
            assert f.byte(5) == 0 and f.byte(3) == fault and f.floating(160) == 0

    def measured_handoff():
        f = Fixture()
        f.start()
        f.advance(300)
        f.range = f.velocity = True
        f.height = 21
        f.advance(150)
        assert f.byte(337) == 1 and f.byte(7) == 0
        f.flow = True
        f.height, f.vz, f.yaw = 250, 80, 5
        for _ in range(24):
            before = f.floating(160)
            f.tick()
            if not f.byte(337):
                assert f.floating(160) == before
                break
        else:
            raise AssertionError('No measured handoff')
        assert f.byte(0) == 3 and f.floating(136) == 250 and f.floating(140) == 800
        assert f.floating(152) == 0 and f.byte(336) == 0 and f.byte(7) == 1
        before = f.floating(160)
        f.tick()
        assert abs(f.floating(160) - before) <= 2.701
        f.range = False
        f.advance(312)
        assert f.byte(0) == 7 and f.byte(3) == 5 and f.byte(337) == 0
        f.tick(4)
        assert f.byte(337) == 0

    def c4_handover():
        f = Fixture()
        f.start()
        f.advance(1002)
        assert f.floating(160) == 400
        f.tick(8)
        f.advance(108)
        assert f.byte(338) == 1 and f.floating(160) == 400
        f.channels[2] = 1445
        f.advance(216)
        assert f.byte(0) == 1 and f.byte(337) == 0 and f.floating(160) == 400
        assert struct.unpack('<H', f.cpu.mem_read(f.core + 46, 2))[0] == 1
        f.advance(1002)
        assert f.byte(5) == 1

    def no_low_restart():
        f = Fixture()
        f.arm()
        f.channels[2] = 1200
        f.tick()
        assert f.byte(336) == 0
        f.channels[2] = 1000
        f.advance(906)
        assert f.byte(5) == 1 and f.byte(6) == 0
        f.range = f.velocity = True
        f.height = 21
        f.tick(4)
        assert f.byte(0) == 1 and f.byte(3) == 8 and f.byte(6) == 0 and f.byte(337) == 0

    tests = [('bounded_timeout', bounded_timeout), ('emergency_and_io', emergency_and_io),
             ('measured_handoff', measured_handoff), ('c4_handover', c4_handover),
             ('no_low_restart', no_low_restart), ('wrap_deadline', lambda: bounded_timeout(0xffffff00))]
    results = []
    for name, test in tests:
        test()
        results.append({'name': name, 'passed': True})
        print('PASS', args.variant, name)
    report = {'passed': True, 'variant': args.variant, 'cases': results,
              'engine': 'unicorn ' + unicorn.__version__, 'replay_step_ms': 6,
              'axf_sha256': hashlib.sha256(axf.read_bytes()).hexdigest(),
              'hex_sha256': hashlib.sha256(hexfile.read_bytes()).hexdigest(),
              'hardware_access': False, 'flashed': False,
              'scope': 'Actual ARM state machine with synthetic sensor/RC frames; no flight dynamics.'}
    (ROOT / 'verification' / ('near_takeoff_' + args.variant + '_results.json')).write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')


if __name__ == '__main__':
    main()
