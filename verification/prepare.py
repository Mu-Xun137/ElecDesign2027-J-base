"""Copy the two specified baselines once, without build products or debugger state."""
from pathlib import Path
import hashlib
import json
import shutil
import zipfile
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
SCHOOL = ROOT.parent
FLIGHT = SCHOOL / '无人机/2024外扩激光光流附加资料包/外扩激光光流模块/主控程序(keil MDK打开)+外扩激光+光流20261001/主控程序(keil MDK打开)+外扩激光+光流20261001'
REMOTE = SCHOOL / '无人机/2024四轴飞行器/4.源程序/遥控程序(keil MDK打开)/遥控程序(keil MDK打开)'

def main():
    manifest = {}
    for label, source in [('flight', FLIGHT), ('remote', REMOTE)]:
        target = ROOT / label
        if target.exists():
            raise RuntimeError(f'Refusing to replace existing delivery: {target}')
        for p in source.rglob('*'):
            rel = p.relative_to(source)
            if any(x.lower() in ('output', 'objects', 'listings', 'debugconfig', '.git', 'rte') for x in rel.parts):
                continue
            if p.is_file() and p.suffix.lower() in ('.c', '.h', '.s', '.uvprojx', '.txt'):
                dest = target / rel
                dest.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(p, dest)
                manifest[f'{label}/{rel.as_posix()}'] = hashlib.sha256(p.read_bytes()).hexdigest()
    for sub in ['docs', 'firmware/basic', 'firmware/upgrade_single', 'firmware/upgrade_ab', 'upgrade/flight']:
        (ROOT / sub).mkdir(parents=True, exist_ok=True)
    report = {'sources': {'flight': str(FLIGHT), 'remote': str(REMOTE)}, 'sha256': manifest}
    (ROOT / 'verification/baseline_manifest.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    with zipfile.ZipFile(SCHOOL / '2026年电子电赛试题.docx') as z:
        doc = ET.fromstring(z.read('word/document.xml'))
    ns = {'w': 'http://schemas.openxmlformats.org/wordprocessingml/2006/main'}
    paragraphs = [''.join(t.text or '' for t in p.findall('.//w:t', ns)) for p in doc.findall('.//w:p', ns)]
    title = '室内小型无人机光源检测与定点悬停（J题）'
    start = paragraphs.index(title)
    end = next(i for i in range(start + 1, len(paragraphs)) if paragraphs[i].startswith('场景感知循迹小车（K题）'))
    (ROOT / 'verification/J题_原文提取.txt').write_text('\n'.join(paragraphs[start:end]), encoding='utf-8')
    print(f'Copied {len(manifest)} source/project files; original baselines unchanged.')

if __name__ == '__main__':
    main()
