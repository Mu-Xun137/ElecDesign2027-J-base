"""Audit built inputs/results, preserve maps, clean ONLY generated caches, and package."""
from pathlib import Path
import argparse
import hashlib
import json
import re
import shutil
import zipfile
from build import hex_memory

ROOT=Path(__file__).resolve().parents[1]
VERIFY=ROOT/'verification'
VERSION='J31-20261027-BH-PA9PA10'

def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def read_json(p):return json.loads(p.read_text(encoding='utf-8'))
def dump(p,obj):p.write_text(json.dumps(obj,ensure_ascii=False,indent=2)+'\n',encoding='utf-8',newline='\n')
def safe_generated_remove(p):
    p=p.resolve()
    allowed=[(ROOT/'verification/_build').resolve(),(ROOT/'flight/Output').resolve(),(ROOT/'remote/Output').resolve()]
    if p not in allowed or not p.is_relative_to(ROOT.resolve()):raise ValueError('Unsafe cleanup target: '+str(p))
    if p.exists():shutil.rmtree(p)

def build_audit():
    reports=[]
    for board,variant in [('flight','basic'),('remote','basic'),('flight','upgrade_single'),('flight','upgrade_ab')]:
        label=board+'_'+variant
        r=read_json(VERIFY/(label+'_build.json'))
        assert r['passed'] and not r['hardware_access'] and not r['flashed']
        assert r['ram_allocated_bytes_including_stack_heap']<=20480 and r['flash_programmed_bytes']<=65536
        assert '5.06 update 7' in r['toolchain']
        for rel,expected in r['source_sha256'].items():
            rel=rel.replace('\\','/')
            p=ROOT/board/rel
            if variant!='basic':
                overlay=ROOT/'upgrade'/board/rel
                if overlay.exists():p=overlay
                if rel=='HAL/j_config.h':
                    text=p.read_text(encoding='utf-8')
                    mode='1' if variant=='upgrade_single' else '2'
                    text=re.sub(r'(#define J_NAV_MODE\s+)\d+',lambda m:m[1]+mode,text)
                    actual=hashlib.sha256(text.encode('utf-8')).hexdigest()
                else:actual=sha(p)
            else:actual=sha(p)
            if actual!=expected:raise ValueError('Build input changed since compilation: '+label+'/'+rel)
        for name,info in r['artifacts'].items():
            p=ROOT/'firmware'/variant/name
            assert sha(p)==info['sha256'] and p.stat().st_size==info['bytes']
            if p.suffix=='.hex':
                memory,sp,reset=hex_memory(p)
                assert len(memory)==r['flash_programmed_bytes']
                if board=='flight':assert not any(0x08007C00<=a<0x08008000 for a in memory)
        log=(VERIFY/(label+'_build.log')).read_text(encoding='utf-8')
        warning_lines=[line for line in log.splitlines() if 'Warning:' in line]
        allowed=['CONFIG\\USB_HID.c','HAL\\ANO_DT.c'] if board=='flight' else ['StartUp\\system_stm32f10x.c']
        if any(not any(a in line for a in allowed) for line in warning_lines):raise ValueError('New compiler warning in '+label)
        source=ROOT/board if variant=='basic' else VERIFY/'_build'/variant/board
        mapfile=source/'Output'/(label+'.map')
        destination=VERIFY/(label+'_memory.map')
        if mapfile.exists():shutil.copy2(mapfile,destination)
        assert destination.is_file()
        reports.append(r)
    return reports

def test_audit():
    expected=['core','protocol','hardware','bh_i2c','nav_single','nav_ab','nav_uncalibrated']
    reports=read_json(VERIFY/'software_results.json')
    assert [r['suite'] for r in reports]==expected
    for r in reports:
        assert r['passed'] and r['cases']>0 and not r['flashed'] and not r['hardware_access']
        if r['compile_warnings'].strip():raise ValueError('Host compile warnings: '+r['suite'])
        for rel,digest in {**r['real_sources'],**r['include_sha256']}.items():
            if sha(ROOT/rel)!=digest:raise ValueError('Source changed since test: '+rel)
    return reports

def original_audit():
    baseline=read_json(VERIFY/'baseline_manifest.json')
    count=0
    for key,digest in baseline['sha256'].items():
        board,rel=key.split('/',1)
        p=Path(baseline['sources'][board])/rel
        if sha(p)!=digest:raise ValueError('Original baseline was modified: '+str(p))
        count+=1
    return count

def parameters():
    text=(ROOT/'flight/HAL/j_config.h').read_text(encoding='utf-8')
    rows=[]
    for line in text.splitlines():
        m=re.match(r'#define\s+(J_\w+)\s+(.+)',line)
        if m:
            value=m[2].split('/*')[0].strip()
            if value:rows.append((m[1],value.replace('|','\\|')))
    lines=['# 集中参数完整初值表','','自动从最终基础 `flight/HAL/j_config.h` 提取。基础 `J_NAV_MODE=0`，升级覆盖配置默认 1、双点选 2；升级 `J_NAV_CALIBRATED=0`。物理/控制值的依据和标定要求见 `参数与标定.md`。','','| 宏 | 最终基础值 |','| --- | --- |']
    lines += [f'| `{k}` | `{v}` |' for k,v in rows]
    (ROOT/'docs/集中参数完整表.md').write_text('\n'.join(lines)+'\n',encoding='utf-8',newline='\n')

def diagnostic_archive():
    directory=VERIFY/'toolchain_probe';directory.mkdir(exist_ok=True)
    source=VERIFY/'probe_clamp.c'
    if source.exists():shutil.move(str(source),str(directory/source.name))
    old=VERIFY/'native/probe'
    if old.exists():
        for name in ['compile.log','results.log','results.json']:
            if (old/name).exists():shutil.copy2(old/name,directory/('observed_'+name))
        if not old.resolve().is_relative_to((VERIFY/'native').resolve()):raise ValueError('Unsafe diagnostic path')
        shutil.rmtree(old)
    (directory/'README.md').write_text('# TinyCC 0.9.27 Win64 调查材料\n\n历史最小复现：运行时表达式作为复杂浮点实参时，`J_Clamp(0+0.08*0*dt,-200,200)` 得到 -200，而先存临时变量再传入得到 0。该结果是测试工具链问题，不是飞行通过证据。观察日志明确保留原失败结论，不纳入最终通过统计。\n\n真实控制源码已拆分中间计算，并由 `height_equilibrium_has_no_collective_drift` 和完整对位测试回归。临时 TRACE/DEBUG 插桩已从交付固件源码移除。\n',encoding='utf-8',newline='\n')

def summary(builds,tests):
    cases=sum(x['cases'] for x in tests);checks=sum(x['assertions'] for x in tests)
    lines=['# 本次构建与软件测试汇总','','版本：'+VERSION+'。未访问硬件、未烧录、未运行电机。','','| 构建 | Flash 实际写入字节 / 65536 | RAM 运行时跨度 / 20480 | 错误 / 警告 |','| --- | ---: | ---: | --- |']
    for r in builds:lines.append(f"| {r['board']}_{r['variant']} | {r['flash_programmed_bytes']} | {r['ram_allocated_bytes_including_stack_heap']} | 0 / {r['warnings']}（原库） |")
    lines+=['','飞控另外保留原标定页 1024 字节。RAM 包含解压后的 RW 地址间隙和 4096 字节保留栈；堆仅在链接保留时计入。HEX 地址、校验、Cortex-M3 启动向量均已检查。','','| 实际 C 测试集 | 用例 | 断言 | 编译宏 |','| --- | ---: | ---: | --- |']
    for r in tests:lines.append(f"| {r['suite']} | {r['cases']} | {r['assertions']} | {', '.join(r['defines']) or '基础默认'} |")
    lines+=['',f'合计 **{cases} 个用例、{checks} 次断言通过**。单/双导航的已标定软件测试显式选择 `J_NAV_CALIBRATED=1`；交付升级 HEX 为保护配置 0，另有未标定拒绝测试。','', '状态机、协议、蜂鸣器、驱动/PWM 桩、理想坐标与光照模型的结论不等于 MCU 固件运行或实机试飞。光流尺度、推力/PID、实际灯声及飞行表现待按说明验证。','','所有构建日志为 `*_build.log`，内含实际工具命令；`*_build.json` 包含全部编译输入哈希、固件哈希与向量；链接资源详情见 `*_memory.map`。逐用例日志见 `native/<suite>/results.log`。']
    (VERIFY/'构建与测试汇总.md').write_text('\n'.join(lines)+'\n',encoding='utf-8',newline='\n')
    return cases,checks

def include_file(p):
    rel=p.relative_to(ROOT)
    if any(x in ('Output','_build','__pycache__') for x in rel.parts):return False
    if p.suffix.lower() in ('.exe','.pyc','.zip'):return False
    return p.is_file()

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--keep-builds',action='store_true',help='Preserve generated Output and _build folders')
    args=parser.parse_args()
    builds=build_audit();tests=test_audit();originals=original_audit()
    overlay={p.relative_to(ROOT/'upgrade').as_posix() for p in (ROOT/'upgrade').rglob('*') if p.is_file()}
    assert overlay=={'flight/HAL/j_config.h','flight/HAL/j_nav.c','README_替换与回退.md'}
    parameters();diagnostic_archive();cases,checks=summary(builds,tests)
    changed=[]
    baseline=read_json(VERIFY/'baseline_manifest.json')['sha256']
    for board in ['flight','remote']:
        for p in (ROOT/board).rglob('*'):
            if include_file(p):
                rel=p.relative_to(ROOT).as_posix();before=baseline.get(rel)
                if sha(p)!=before:changed.append({'file':rel,'status':'modified' if before else 'added','sha256':sha(p)})
    dump(VERIFY/'源码变更清单.json',changed)
    if not args.keep_builds:
        for generated in [VERIFY/'_build',ROOT/'flight/Output',ROOT/'remote/Output']:safe_generated_remove(generated)
    audit={'version':VERSION,'delivery_date':'2026-10-27','passed':True,'original_source_files_verified_unchanged':originals,
           'builds':[{'name':r['board']+'_'+r['variant'],'flash_bytes':r['flash_programmed_bytes'],
                      'ram_runtime_bytes':r['ram_allocated_bytes_including_stack_heap'],'warnings':r['warnings']} for r in builds],
           'new_compiler_warnings':0,'software_cases_passed':cases,'software_assertions_passed':checks,
           'upgrade_overlay_files':sorted(overlay),'upgrade_calibrated_gate':0,'hardware_access':False,'flashed':False,'motors_run':False,
           'real_flight_verified':False,'source_firmware_match':True,'baseline_preserved':True,
           'bh1750_wiring':{'scl':'UART1/TX1/PA9','sda':'UART1/RX1/PA10','vcc':'3V3','gnd':'GND','addr':'GND'},
           'archive':ROOT.name+'.zip','archive_hash_file':ROOT.name+'.zip.sha256',
           'limitations':['LED实物映射未核验','推力/PID及阈值待标定','LC08增量更新和尺度待核验','升级导航实机未验证','无线C2非独立硬件急停','无机载蜂鸣/备用高度/自动降落/可靠低电量保护']}
    dump(VERIFY/'最终核验.json',audit)
    manifest=VERIFY/'文件SHA256.json'
    files=[p for p in sorted(ROOT.rglob('*')) if include_file(p) and p!=manifest]
    for p in files:
        if not p.resolve().is_relative_to(ROOT.resolve()):raise ValueError('Nonlocal artifact')
    dump(manifest,{'version':VERSION,'algorithm':'SHA256','self_excluded':True,'files':{
        p.relative_to(ROOT).as_posix():{'bytes':p.stat().st_size,'sha256':sha(p)} for p in files}})
    archive=ROOT.with_name(ROOT.name+'.zip')
    with zipfile.ZipFile(archive,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=6) as z:
        for p in files+[manifest]:z.write(p,ROOT.name+'/'+p.relative_to(ROOT).as_posix())
    with zipfile.ZipFile(archive) as z:
        assert z.testzip() is None
        for rel,entry in read_json(manifest)['files'].items():
            assert hashlib.sha256(z.read(ROOT.name+'/'+rel)).hexdigest()==entry['sha256']
    digest=sha(archive)
    archive.with_suffix('.zip.sha256').write_text(digest+'  '+archive.name+'\n',encoding='utf-8',newline='\n')
    print(json.dumps({'passed':True,'cases':cases,'assertions':checks,'original_files_unchanged':originals,
                      'archive':str(archive),'archive_bytes':archive.stat().st_size,'archive_sha256':digest,
                      'builds':audit['builds']},ensure_ascii=False,indent=2))

if __name__=='__main__':main()
