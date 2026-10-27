"""One-time safe integration of new units into the copied Keil projects."""
from pathlib import Path
import shutil
import re
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]

def read(p):
    b=p.read_bytes()
    for enc in ('utf-8-sig','gb18030'):
        try: return re.sub(r'\r+\n','\n',b.decode(enc)).replace('\r','\n')
        except UnicodeDecodeError: pass
    raise ValueError(p)

def shared():
    for relative in ['HAL/j_config.h','HAL/j_protocol.h','HAL/j_protocol.c','HAL/j_boot.h','HAL/j_boot.c','HARDWARE/nrf24l01.c']:
        shutil.copy2(ROOT/'flight'/relative,ROOT/'remote'/relative)

def project(board,name,units):
    path=ROOT/board/(name+'.uvprojx')
    tree=ET.parse(path); target=tree.find('.//Target'); options=target.find('TargetOption')
    target.find('pCCUsed').text='5060960::V5.06 update 7 (build 960)::.\\ARMCC'
    target.find('uAC6').text='0'
    for item in tree.findall('.//RunUserProg1')+tree.findall('.//RunUserProg2'):
        if item.text not in ('0',None): raise RuntimeError('Unexpected build hook: '+str(item.text))
    common=options.find('TargetCommonOption')
    common.find('OutputDirectory').text='.\\Output\\'
    common.find('OutputName').text='flight_basic' if board=='flight' else 'remote_basic'
    common.find('CreateHexFile').text='1'
    arm=options.find('TargetArmAds'); arm.find('Cads/uC99').text='1'; arm.find('Cads/Optim').text='3'
    defines=arm.find('Cads/VariousControls/Define')
    if board=='remote' and 'J_REMOTE' not in (defines.text or ''): defines.text=(defines.text or '')+',J_REMOTE'
    if board=='flight':
        ld=arm.find('LDads'); ld.find('useFile').text='1'; ld.find('ScatterFile').text='.\\j_memory.sct'
    groups=target.find('Groups')
    for g in list(groups):
        if g.findtext('GroupName')=='JTask': groups.remove(g)
    g=ET.SubElement(groups,'Group'); ET.SubElement(g,'GroupName').text='JTask'
    fs=ET.SubElement(g,'Files')
    for rel in units:
        file=ET.SubElement(fs,'File')
        ET.SubElement(file,'FileName').text=Path(rel).name
        ET.SubElement(file,'FileType').text='1'
        ET.SubElement(file,'FilePath').text='.\\'+rel.replace('/','\\')
    ET.indent(tree,space='  '); tree.write(path,encoding='utf-8',xml_declaration=True)

def main():
    data=ROOT/'flight/USER/ALL_DATA.h'; text=read(data)
    if '#include <stdint.h>' not in text:
        start=text.index('typedef'); end=text.index('extern volatile uint32_t SysTick_count;')
        text=text[:start]+'#include <stdint.h>\n\n'+text[end:]
    if 'prevMeasured' not in text:
        text=text.replace('float OutLimitLow;','float OutLimitLow;\n\tfloat prevMeasured;\n\tuint8_t initialized;')
    data.write_text(text,encoding='utf-8',newline='\n')
    sys=ROOT/'remote/USER/sys.h'; text=read(sys)
    text=text.replace('extern uint32_t SysTick_count','extern volatile uint32_t SysTick_count')
    sys.write_text(text,encoding='utf-8',newline='\n')
    imu=ROOT/'flight/MATH/imu.c'; text=read(imu)
    for axis in 'xyz':
        old=f'GyroIntegError.{axis} += AccGravity.{axis} * KiDef;'
        text=text.replace(old,f'GyroIntegError.{axis} += AccGravity.{axis} * KiDef * (dt/0.00626f);')
    text=text.replace('-asin(vecxZ)', '-asin(vecxZ>1.0f?1.0f:(vecxZ<-1.0f?-1.0f:vecxZ))')
    imu.write_text(text,encoding='utf-8',newline='\n')
    # On an unrecoverable exception clear all four PWM registers before trapping.
    it=ROOT/'flight/StartUp/stm32f10x_it.c'; text=read(it)
    for handler in ['HardFault_Handler','MemManage_Handler','BusFault_Handler','UsageFault_Handler','NMI_Handler']:
        signature=f'void {handler}(void)'
        pos=text.find(signature)
        if pos>=0:
            brace=text.find('{',pos)
            if 'TIM2->CCR1=0' not in text[brace:brace+150]:
                text=text[:brace+1]+'\n  TIM2->CCR1=0; TIM2->CCR2=0; TIM2->CCR3=0; TIM2->CCR4=0;\n'+text[brace+1:]
    it.write_text(text,encoding='utf-8',newline='\n')
    shared()
    startup=ROOT/'flight/StartUp/startup_stm32f10x_md.s'
    text=read(startup)
    text=re.sub(r'(Stack_Size\s+EQU\s+)0x[0-9A-Fa-f]+',r'\g<1>0x00001000',text)
    startup.write_text(text,encoding='utf-8',newline='\n')
    project('flight','mini',['HAL/j_core.c','HAL/j_nav.c','HAL/j_protocol.c','HAL/j_sensors.c','HAL/j_platform.c','HAL/j_boot.c','HARDWARE/j_bh1750.c'])
    project('remote','remote',['HAL/j_protocol.c','HAL/j_keys.c','HAL/j_feedback.c','HAL/j_boot.c'])
    print('Keil projects updated; build hooks disabled; ARMCC5/C99/O2; shared protocol synchronized.')

if __name__=='__main__': main()
