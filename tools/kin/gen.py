#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Jack Sidman Smith
#
# Writes table.h (and rows.txt, the same rows as text): kinematics inputs (FK, IK, steps, legality) as float32 bit patterns. Usage: gen.py <seed>

import random, struct, sys
seed=int(sys.argv[1]); random.seed(seed)
def f(x): return struct.unpack('<f',struct.pack('<f',x))[0]
def b(x): return struct.unpack('<I',struct.pack('<f',x))[0]
rows=[]
for _ in range(600): rows.append(("A",f(random.uniform(-25,140)),f(random.uniform(-25,125)),f(random.uniform(-95,95))))
for _ in range(600): rows.append(("C",f(random.uniform(-50,400)),f(random.uniform(-350,350)),f(random.uniform(-150,350))))
if seed==1:
    for x in (0.0,-0.0,-1.0,-100.0):
        for y in (0.0,5.0,-5.0): rows.append(("C",f(x),f(y),f(100.0)))
for _ in range(400): rows.append(("S",f(random.uniform(-100,200)),f(random.uniform(-30,140)),f(random.uniform(0,180))))
for _ in range(200):
    a=random.choice([135.6,-20.0,100.0,30.0]); bb=random.choice([119.9,-20.0,50.0,90.0]); c=random.choice([90.0,-90.0,0.0])
    rows.append(("L",f(a+random.uniform(-0.01,0.01)),f(bb+random.uniform(-0.01,0.01)),f(c+random.uniform(-0.01,0.01))))
with open("table.h","w") as o:
    o.write("static const struct Row kRows[] PROGMEM = {\n")
    o.write(",\n".join("{'%s',{0x%08xUL,0x%08xUL,0x%08xUL}}"%(k,b(x),b(y),b(z)) for k,x,y,z in rows))
    o.write("};\n")
with open("rows.txt","w") as o:
    o.write("\n".join("%s %08x %08x %08x"%(k,b(x),b(y),b(z)) for k,x,y,z in rows)+"\n")
