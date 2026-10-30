"""Execute the real ARM JSensors_Module with measured millimetre frames.

Requires unicorn==2.1.4. This checks compiled conversion/validity behaviour,
not laser hardware, optical-flow dynamics, or flight performance.
"""
from pathlib import Path
import argparse
import hashlib
import json
import struct
import sys

from build import ROOT, hex_memory
from verify_timer import symbol_address


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--variant', choices=['basic', 'upgrade_single', 'upgrade_ab'], default='basic')
    parser.add_argument('--dependency-path', type=Path)
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    if args.dependency_path:
        sys.path.insert(0, str(args.dependency_path.resolve()))
    import unicorn
    from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS
    from unicorn.arm_const import UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_R3
    from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_MSP, UC_ARM_REG_LR, UC_ARM_REG_PC

    axf = ROOT / 'firmware' / args.variant / ('flight_' + args.variant + '.axf')
    hexfile = axf.with_suffix('.hex')
    memory, _, _ = hex_memory(hexfile)
    image = bytearray(b'\xff' * 0x10000)
    for address, byte in memory.items():
        image[address - 0x08000000] = byte
    entry = symbol_address(axf, 'JSensors_Module')
    results = []
    for raw_mm, valid in [(0, False), (294, True), (800, True), (2000, True), (3500, True), (3501, False)]:
        cpu = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
        cpu.mem_map(0x08000000, 0x10000)
        cpu.mem_write(0x08000000, bytes(image))
        cpu.mem_map(0x20000000, 0x5000)
        cpu.mem_map(0x10000000, 0x1000)
        sensor, stack = 0x20003000, 0x20004f00
        for now in [1000, 1030]:
            cpu.reg_write(UC_ARM_REG_MSP, stack)
            cpu.reg_write(UC_ARM_REG_SP, stack)
            cpu.reg_write(UC_ARM_REG_LR, 0x10000001)
            cpu.reg_write(UC_ARM_REG_R0, sensor)
            cpu.reg_write(UC_ARM_REG_R1, raw_mm)
            cpu.reg_write(UC_ARM_REG_R2, 0)
            cpu.reg_write(UC_ARM_REG_R3, 0)
            cpu.mem_write(stack, struct.pack('<III', 80, 1, now))
            cpu.emu_start(entry | 1, 0x10000000, timeout=1000000, count=100000)
            if cpu.reg_read(UC_ARM_REG_PC) != 0x10000000:
                raise RuntimeError('JSensors_Module did not return')
        data = cpu.mem_read(sensor, 88)
        height = struct.unpack_from('<f', data, 20)[0]
        range_ok, velocity_ok, flow_ok = data[76:79]
        module_ms = struct.unpack_from('<I', data, 84)[0]
        expected_height = raw_mm if valid else 0
        passed = height == expected_height and [range_ok, velocity_ok, flow_ok] == [int(valid)] * 3 and module_ms == 1030
        results.append({'raw_mm': raw_mm, 'expected_height_mm': expected_height, 'height_mm': height,
                        'range_ok': range_ok, 'velocity_ok': velocity_ok, 'flow_ok': flow_ok,
                        'expected_valid': valid, 'module_ms': module_ms, 'passed': passed})
        print(f'{"PASS" if passed else "FAIL"} {args.variant}: raw={raw_mm}, height_mm={height:g}, flags={range_ok}/{velocity_ok}/{flow_ok}')
    report = {'passed': all(r['passed'] for r in results), 'variant': args.variant,
              'engine': 'unicorn ' + unicorn.__version__, 'entry': hex(entry),
              'axf_sha256': hashlib.sha256(axf.read_bytes()).hexdigest(),
              'hex_sha256': hashlib.sha256(hexfile.read_bytes()).hexdigest(), 'cases': results,
              'hardware_access': False, 'flashed': False,
              'scope': 'Compiled ARM input conversion/validity only; no physical flight model.'}
    destination = args.report or ROOT / 'verification' / ('range_units_' + args.variant + '_results.json')
    destination.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
