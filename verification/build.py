"""Clean ARMCC5 build of the copied Keil projects. Never loads a probe or downloads.
The commands are recorded in each log. uvprojx is the source of the source/include list.
"""
from pathlib import Path
import argparse
import hashlib
import json
import re
import shutil
import struct
import subprocess
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]
TOOLS=Path('E:/Keil5/Core/ARM/ARMCC/Bin')

def run(cmd,cwd,log):
    log.write('COMMAND '+subprocess.list2cmdline([str(x) for x in cmd])+'\n'); log.flush()
    p=subprocess.run([str(x) for x in cmd],cwd=cwd,capture_output=True,timeout=90,
                     creationflags=subprocess.CREATE_NO_WINDOW)
    text=(p.stdout+p.stderr).decode('gb18030',errors='replace')
    log.write(text); log.flush()
    if p.returncode:
        raise RuntimeError(f'{Path(str(cmd[0])).name} exited {p.returncode}:\n{text[-6000:]}')
    return text

def hex_memory(path):
    memory={};base=0;eof=False
    for line in path.read_text(encoding='ascii').splitlines():
        if eof or not line.startswith(':'): raise ValueError('Invalid HEX record')
        b=bytes.fromhex(line[1:])
        if len(b)!=b[0]+5 or sum(b)%256: raise ValueError('Invalid HEX length/checksum')
        kind=b[3];addr=int.from_bytes(b[1:3],'big')
        if kind==0:
            for k,x in enumerate(b[4:-1]):
                where=base+addr+k
                if not 0x08000000<=where<0x08010000 or where in memory: raise ValueError('Flash overflow/overlap')
                memory[where]=x
        elif kind==1: eof=True
        elif kind==4: base=int.from_bytes(b[4:-1],'big')<<16
        elif kind!=5: raise ValueError(f'Unsupported HEX type {kind}')
    if not eof or not memory or min(memory)!=0x08000000: raise ValueError('Missing image/EOF')
    vector=bytes(memory[a] for a in range(0x08000000,0x08000008))
    sp,reset=struct.unpack('<II',vector)
    if not (0x20000000<sp<=0x20005000 and sp%8==0 and reset&1 and (reset&~1) in memory):
        raise ValueError('Invalid STM32F103C8 boot vectors')
    return memory,sp,reset

def ram_allocated(elf):
    data=elf.read_bytes()
    if data[:5]!=b'\x7fELF\x01': raise ValueError('Not ELF32')
    header=struct.unpack_from('<16sHHIIIIIHHHHHH',data)
    offset,entry_size,count=header[6],header[11],header[12]
    end=0x20000000; regions=[]
    for k in range(count):
        _,kind,flags,addr,_,size,*_=struct.unpack_from('<IIIIIIIIII',data,offset+k*entry_size)
        if flags&2 and 0x20000000<=addr<0x20005000:
            if addr+size>0x20005000: raise ValueError('20 KiB RAM overflow')
            end=max(end,addr+size);regions.append([hex(addr),size,'ZI' if kind==8 else 'RW'])
    # ARM ELF may store a COMPRESSED RW payload with a smaller sh_size than the
    # runtime data region. The highest RAM address includes the decompression gap.
    return end-0x20000000,regions

def build(board,variant):
    original=ROOT/board
    work=original
    if variant!='basic':
        work=ROOT/'verification/_build'/variant/board
        if work.exists():
            if not work.resolve().is_relative_to((ROOT/'verification/_build').resolve()): raise ValueError('Unsafe cleanup path')
            shutil.rmtree(work)
        shutil.copytree(original,work,ignore=shutil.ignore_patterns('Output'))
        for p in (ROOT/'upgrade'/board).rglob('*'):
            if p.is_file():
                dest=work/p.relative_to(ROOT/'upgrade'/board);dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,dest)
        config=work/'HAL/j_config.h'
        text=config.read_text(encoding='utf-8')
        text=re.sub(r'(#define J_NAV_MODE\s+)\d+',lambda m:m[1]+('1' if variant=='upgrade_single' else '2'),text)
        config.write_text(text,encoding='utf-8',newline='\n')
    project=work/('mini.uvprojx' if board=='flight' else 'remote.uvprojx')
    tree=ET.parse(project)
    for node in tree.findall('.//RunUserProg1')+tree.findall('.//RunUserProg2'):
        if node.text not in ('0',None): raise ValueError('Build hooks must remain disabled')
    arm=tree.find('.//TargetArmAds')
    if tree.findtext('.//Device')!='STM32F103C8' or tree.findtext('.//uAC6')=='1': raise ValueError('Wrong MCU/toolchain')
    source=[]
    for node in tree.findall('.//FilePath'):
        p=(work/node.text.replace('\\','/')).resolve()
        if not p.is_relative_to(work.resolve()) or not p.is_file(): raise ValueError(f'Missing/nonlocal source: {node.text}')
        if p.suffix.lower() in ('.c','.s'): source.append(p)
    out=work/'Output'
    if out.exists():
        if not out.resolve().is_relative_to(work.resolve()): raise ValueError('Unsafe output path')
        shutil.rmtree(out)
    out.mkdir()
    label=f'{board}_{variant}'
    log_path=ROOT/'verification'/(label+'_build.log')
    inc=(arm.findtext('Cads/VariousControls/IncludePath') or '').split(';')
    defines=(arm.findtext('Cads/VariousControls/Define') or '').split(',')
    with log_path.open('w',encoding='utf-8') as log:
        version=run([TOOLS/'armcc.exe','--vsn'],work,log)
        cflags=['--cpu','Cortex-M3','--c99','--apcs=interwork','--split_sections','--library_type=microlib','-O2','-g']
        for item in inc:
            if item:cflags+=['-I',item]
        for item in defines:
            if item:cflags+=['-D',item.strip()]
        objects=[]
        for k,p in enumerate(source):
            obj=out/(f'{k:03d}_'+p.stem+'.o');objects.append(obj)
            rel=str(p.relative_to(work))
            if p.suffix.lower()=='.c': cmd=[TOOLS/'armcc.exe',*cflags,'-c',rel,'-o',str(obj.relative_to(work))]
            else: cmd=[TOOLS/'armasm.exe','--cpu','Cortex-M3','--apcs=interwork','--pd','__MICROLIB SETA 1','-g',rel,'-o',str(obj.relative_to(work))]
            run(cmd,work,log)
        axf=out/(label+'.axf'); mapfile=out/(label+'.map')
        scatter=work/'j_memory.sct'
        if not scatter.exists():
            scatter.write_text('LR_FLASH 0x08000000 0x10000 {\n ER_FLASH 0x08000000 0x10000 {\n *.o (RESET,+First)\n *(InRoot$$Sections)\n .ANY (+RO)\n }\n RW_RAM 0x20000000 0x5000 { .ANY (+RW +ZI) }\n}\n',encoding='ascii')
        link=[TOOLS/'armlink.exe','--cpu','Cortex-M3','--library_type=microlib','--entry','Reset_Handler',
              '--scatter',str(scatter.relative_to(work)),'--strict','--map','--symbols','--xref',
              '--info','sizes,totals,unused,veneers','--list',str(mapfile.relative_to(work)),
              '--output',str(axf.relative_to(work)),*[str(o.relative_to(work)) for o in objects]]
        run(link,work,log)
        hexfile=out/(label+'.hex')
        run([TOOLS/'fromelf.exe','--i32combined','--output',str(hexfile.relative_to(work)),str(axf.relative_to(work))],work,log)
    memory,sp,reset=hex_memory(hexfile)
    if board=='flight' and any(0x08007C00<=a<0x08008000 for a in memory): raise ValueError('IMU calibration page overlaps code')
    ram,regions=ram_allocated(axf)
    dest=ROOT/'firmware'/variant;dest.mkdir(exist_ok=True)
    artifacts={}
    for p in (hexfile,axf):
        to=dest/p.name;shutil.copy2(p,to)
        artifacts[p.name]={'bytes':to.stat().st_size,'sha256':hashlib.sha256(to.read_bytes()).hexdigest()}
    logtext=log_path.read_text(encoding='utf-8')
    report={'passed':True,'board':board,'variant':variant,'toolchain':version.strip(),
            'source_project':str(project.relative_to(ROOT)),'source_sha256':{
                str(p.relative_to(work)):hashlib.sha256(p.read_bytes()).hexdigest() for p in work.rglob('*')
                if p.is_file() and p.suffix.lower() in ('.c','.h','.s','.sct','.uvprojx') and 'Output' not in p.parts},
            'flash_programmed_bytes':len(memory),'flash_capacity_bytes':65536,
            'ram_allocated_bytes_including_stack_heap':ram,'ram_capacity_bytes':20480,'ram_regions':regions,
            'ram_method':'highest allocated RAM end minus 0x20000000; includes decompressed RW gaps and retained stack/heap',
            'flash_span_bytes':max(memory)-min(memory)+1,
            'initial_sp':hex(sp),'reset_vector':hex(reset),'calibration_page_reserved':board=='flight',
            'warnings':len(re.findall(r'warning:',logtext,re.I)),'artifacts':artifacts,
            'flashed':False,'hardware_access':False}
    (ROOT/'verification'/(label+'_build.json')).write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(f'{label}: PASS, Flash {len(memory)}/65536, RAM {ram}/20480, warnings {report["warnings"]}',flush=True)
    return report

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--variant',choices=['basic','upgrade_single','upgrade_ab'],default='basic')
    p.add_argument('--board',choices=['flight','remote','both'],default='both')
    args=p.parse_args()
    boards=['flight','remote'] if args.board=='both' else [args.board]
    for board in boards: build(board,args.variant)

if __name__=='__main__':main()
