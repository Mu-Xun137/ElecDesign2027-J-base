"""Run the actual ARM TIM1_Config machine code with dirty stack/register state.

Requires unicorn==2.1.4. MMIO is register-backed memory, not a timing model:
the update period is calculated from the resulting PSC/ARR/RCR at 72 MHz.
No debug probe, flashing, or physical motor access is used.
"""
from pathlib import Path
import argparse
import hashlib
import json
import struct
import sys

from build import ROOT, hex_memory


def symbol_address(path, name):
    data = path.read_bytes()
    if data[:6] != b'\x7fELF\x01\x01':
        raise ValueError('Expected little-endian ELF32')
    header = struct.unpack_from('<16sHHIIIIIHHHHHH', data)
    sections = [struct.unpack_from('<10I', data, header[6] + k * header[11])
                for k in range(header[12])]
    for section in sections:
        if section[1] != 2:
            continue
        strings = sections[section[6]]
        names = data[strings[4]:strings[4] + strings[5]]
        for offset in range(section[4], section[4] + section[5], section[9]):
            index, value, _, _, _, _ = struct.unpack_from('<IIIBBH', data, offset)
            end = names.find(b'\0', index)
            if names[index:end].decode('ascii', errors='replace') == name:
                return value
    raise ValueError('Missing ELF symbol: ' + name)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--variant', choices=['basic', 'upgrade_single', 'upgrade_ab'], default='basic')
    parser.add_argument('--axf', type=Path)
    parser.add_argument('--hex', type=Path, dest='hexfile')
    parser.add_argument('--dependency-path', type=Path)
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    if args.dependency_path:
        sys.path.insert(0, str(args.dependency_path.resolve()))
    import unicorn
    from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS
    from unicorn import arm_const
    from unicorn.arm_const import UC_ARM_REG_MSP, UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_PC

    axf = args.axf or ROOT / 'firmware' / args.variant / ('flight_' + args.variant + '.axf')
    hexfile = args.hexfile or axf.with_suffix('.hex')
    report_path = args.report or ROOT / 'verification' / ('timer_' + args.variant + '_results.json')
    memory, initial_sp, _ = hex_memory(hexfile)
    entry = symbol_address(axf, 'TIM1_Config')
    image = bytearray(b'\xff' * 0x10000)
    for address, value in memory.items():
        image[address - 0x08000000] = value
    results = []
    for pattern in [0x00, 0x5A, 0xA5, 0xFF]:
        cpu = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
        cpu.mem_map(0x08000000, 0x10000)
        cpu.mem_write(0x08000000, bytes(image))
        cpu.mem_map(0x20000000, 0x5000)
        cpu.mem_write(0x20000000, bytes([pattern]) * 0x5000)
        cpu.mem_map(0x40000000, 0x30000)
        cpu.mem_map(0xE000E000, 0x2000)
        sentinel = 0x10000000
        cpu.mem_map(sentinel, 0x1000)
        # main() selects NVIC_PriorityGroup_2 before TIM1_Config().
        cpu.mem_write(0xE000ED0C, struct.pack('<I', 0x05FA0500))
        cpu.mem_write(0x40012C30, struct.pack('<I', 90))
        # ARMCC may reserve locals with PUSH {r0-r4,lr}, so an uninitialized
        # member can inherit an incoming register as well as old stack bytes.
        for reg in range(13):
            cpu.reg_write(getattr(arm_const, 'UC_ARM_REG_R' + str(reg)), pattern * 0x01010101)
        cpu.reg_write(UC_ARM_REG_MSP, initial_sp)
        cpu.reg_write(UC_ARM_REG_SP, initial_sp)
        cpu.reg_write(UC_ARM_REG_LR, sentinel | 1)
        cpu.emu_start(entry | 1, sentinel, timeout=1000000, count=10000)
        if cpu.reg_read(UC_ARM_REG_PC) != sentinel:
            raise RuntimeError('TIM1_Config did not return within the instruction limit')
        def register(address):
            return struct.unpack('<I', cpu.mem_read(address, 4))[0]
        psc, arr, rcr = [register(a) for a in [0x40012C28, 0x40012C2C, 0x40012C30]]
        period_us = (psc + 1) * (arr + 1) * (rcr + 1) * 1000000 // 72000000
        enabled = bool(register(0x40012C00) & 1 and register(0x40012C0C) & 1
                       and register(0xE000E100) & (1 << 25))
        passed = rcr == 0 and period_us == 3000 and enabled
        results.append({'initial_stack_and_register_pattern': hex(pattern), 'psc': psc, 'arr': arr, 'rcr': rcr,
                        'calculated_update_period_us': period_us, 'irq_enabled': enabled, 'passed': passed})
        print(f'{"PASS" if passed else "FAIL"} initial_state=0x{pattern:02X}: RCR={rcr}, period={period_us} us, IRQ={enabled}')
    report = {'passed': all(r['passed'] for r in results), 'engine': 'unicorn ' + unicorn.__version__,
              'axf': str(axf.resolve()), 'hex': str(hexfile.resolve()),
              'axf_sha256': hashlib.sha256(axf.read_bytes()).hexdigest(),
              'hex_sha256': hashlib.sha256(hexfile.read_bytes()).hexdigest(),
              'entry': hex(entry), 'cases': results, 'hardware_access': False, 'flashed': False,
              'scope': 'Real ARM initialization code; register memory fixture, not physical timing or flight verification.'}
    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
