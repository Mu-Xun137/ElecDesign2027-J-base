"""Compile and execute the delivered C modules with deterministic sensor/clock/PWM fixtures."""
from pathlib import Path
import argparse
import hashlib
import json
import re
import subprocess

ROOT=Path(__file__).resolve().parents[1]
TCC=ROOT.parent/'27-diansai/验证/工具/tcc/tcc.exe'

def run_suite(name,units,defines=()):
    units=[*units,'verification/native_math.c']
    out=ROOT/'verification/native'/name;out.mkdir(parents=True,exist_ok=True)
    exe=out/'tests.exe'
    cmd=[str(TCC),'-std=c99','-Wall','-I'+str(ROOT/'flight/HAL'),'-I'+str(ROOT/'remote/HAL')]
    if name in ('hardware','bh_i2c'):
        cmd[1:1]=['-I'+str(ROOT/'verification/stubs'),'-I'+str(ROOT/'flight/HARDWARE'),'-I'+str(ROOT/'flight/MATH'),'-I'+str(ROOT/'flight/USER'),
                  '-include',str(ROOT/'verification/stubs/ALL_DEFINE.h')]
    cmd+=['-D'+x for x in defines]
    cmd+=['-o',str(exe),*[str(ROOT/u) for u in units]]
    build=subprocess.run(cmd,cwd=ROOT,capture_output=True,timeout=60,creationflags=subprocess.CREATE_NO_WINDOW)
    buildtext=(build.stdout+build.stderr).decode('gb18030',errors='replace')
    (out/'compile.log').write_text(subprocess.list2cmdline(cmd)+'\n'+buildtext,encoding='utf-8')
    if build.returncode: raise RuntimeError(buildtext)
    p=subprocess.run([str(exe)],cwd=ROOT,capture_output=True,timeout=60,creationflags=subprocess.CREATE_NO_WINDOW)
    text=(p.stdout+p.stderr).decode('utf-8',errors='replace')
    (out/'results.log').write_text(text,encoding='utf-8')
    print(text,flush=True)
    m=re.search(r'RESULT cases=(\d+) checks=(\d+)',text)
    report={'suite':name,'passed':p.returncode==0 and bool(m),'cases':int(m[1]) if m else 0,
            'assertions':int(m[2]) if m else 0,'compiler':str(TCC),'compile_warnings':buildtext,
            'real_sources':{u:hashlib.sha256((ROOT/u).read_bytes()).hexdigest() for u in units},'defines':list(defines),
            'include_sha256':{str(h.relative_to(ROOT)):hashlib.sha256(h.read_bytes()).hexdigest()
                for base in [ROOT/'flight',ROOT/'remote',ROOT/'upgrade/flight',ROOT/'verification/stubs'] for h in base.rglob('*.h')},
            'firmware_executed_on_mcu':False,'hardware_access':False,'flashed':False}
    (out/'results.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    if not report['passed']: raise RuntimeError(f'{name} failed; see {out}/results.log')
    return report

def main():
    global TCC
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--suite',default='core');p.add_argument('--tcc',default=str(TCC));args=p.parse_args()
    TCC=Path(args.tcc).resolve()
    suites={
      'core':['verification/test_core.c','flight/HAL/j_core.c','flight/HAL/j_sensors.c','flight/HAL/j_nav.c'],
      'protocol':['verification/test_protocol.c','flight/HAL/j_protocol.c','remote/HAL/j_keys.c','remote/HAL/j_feedback.c'],
      'hardware':['verification/test_hardware.c','flight/HAL/j_core.c','flight/HAL/j_sensors.c','flight/HAL/j_nav.c',
        'flight/HAL/j_protocol.c','flight/HAL/j_platform.c','flight/HAL/control.c','flight/HARDWARE/flow.c',
        'flight/HARDWARE/j_bh1750.c','flight/HARDWARE/mpu6050.c','flight/MATH/pid.c','flight/USER/delay.c','flight/USER/scheduler.c'],
      'bh_i2c':['verification/test_bh_i2c.c','flight/HARDWARE/j_bh_i2c.c'],
      'nav_single':['verification/test_nav.c','flight/HAL/j_core.c','upgrade/flight/HAL/j_nav.c'],
      'nav_ab':['verification/test_nav.c','flight/HAL/j_core.c','upgrade/flight/HAL/j_nav.c'],
      'nav_uncalibrated':['verification/test_nav.c','flight/HAL/j_core.c','upgrade/flight/HAL/j_nav.c'],
    }
    selected=suites.keys() if args.suite=='all' else [args.suite]
    reports=[]
    for s in selected:
        defines=[]
        if s.startswith('nav_'):defines=['J_NAV_MODE='+('2' if s=='nav_ab' else '1'),'J_NAV_CALIBRATED='+('0' if s=='nav_uncalibrated' else '1')]
        reports.append(run_suite(s,suites[s],defines))
    (ROOT/'verification/software_results.json').write_text(json.dumps(reports,ensure_ascii=False,indent=2),encoding='utf-8')

if __name__=='__main__':main()
