#!/usr/bin/env python3
"""Make simulation copies of the core sources in which every register without
an initial value starts at 0 (as the FPGA flip-flops do after configuration).
Also replaces the blocking assignments in spi.v's clocked always blocks (a
simulation race only)."""
import re, os, glob
os.makedirs('gen', exist_ok=True)
decl = re.compile(r'^(\s*(?:output\s+)?reg\s+(?:signed\s+)?(?:\[[^\]]+\]\s*)?)([^;=\[]+?)(\s*[;,])\s*(//.*)?$')
for f in glob.glob('../*.v'):
    out = []
    in_sub = False
    for line in open(f):
        if re.match(r'^\s*(task|function)\b', line): in_sub = True
        if re.match(r'^\s*(endtask|endfunction)\b', line): in_sub = False
        m = decl.match(line.rstrip('\n'))
        if m and not in_sub and '[' not in m.group(2) and '=' not in line:
            names = [n.strip() for n in m.group(2).split(',')]
            if all(re.match(r'^[A-Za-z_]\w*$', n) for n in names):
                line = m.group(1) + ', '.join(n + ' = 0' for n in names) + m.group(3) + (' ' + m.group(4) if m.group(4) else '') + '\n'
        out.append(line)
    s = ''.join(out)
    if f.endswith('spi.v'):
        s = s.replace('cmd_ready_r2 = byte', 'cmd_ready_r2 <= byte').replace('param_ready_r2 = byte', 'param_ready_r2 <= byte')
    if f.endswith('msu.v'):
        s = s.replace('status_reset_we_r = {', 'status_reset_we_r <= {')
    open(os.path.join('gen', os.path.basename(f)), 'w').write(s)
