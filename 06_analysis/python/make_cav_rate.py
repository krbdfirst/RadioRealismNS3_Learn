#!/usr/bin/env python3
"""
Generate an intersection_cav<RATE>.rou.xml at an arbitrary CAV penetration by
rescaling the veh_av : veh_human flow-probability split of an existing rate file.

Penetration is defined per route as p_av / (p_av + p_human); the total motorized
demand (p_av + p_human) and the bike probability are held constant, so only the
AV/human split changes — identical to how the existing cavNN files differ.

Usage: python3 make_cav_rate.py --rate 40 --src intersection_cav50.rou.xml
"""
import argparse, re, sys
import xml.etree.ElementTree as ET

ap = argparse.ArgumentParser()
ap.add_argument("--rate", type=int, required=True, help="target CAV %% (e.g. 40)")
ap.add_argument("--src", default="intersection_cav50.rou.xml", help="source rate file")
ap.add_argument("--out", default=None)
a = ap.parse_args()
frac = a.rate / 100.0
out = a.out or f"intersection_cav{a.rate:02d}.rou.xml"

tree = ET.parse(a.src)
root = tree.getroot()

# group av/human flows by route
by_route = {}
for fl in root.findall("flow"):
    if fl.get("type") in ("veh_av", "veh_human"):
        by_route.setdefault(fl.get("route"), {})[fl.get("type")] = fl

changed = 0
for route, pair in by_route.items():
    av, hu = pair.get("veh_av"), pair.get("veh_human")
    if av is None or hu is None:
        continue
    total = float(av.get("probability")) + float(hu.get("probability"))
    av.set("probability", f"{frac * total:.8f}")
    hu.set("probability", f"{(1.0 - frac) * total:.8f}")
    changed += 1

tree.write(out, encoding="UTF-8", xml_declaration=True)
print(f"wrote {out}: {changed} routes rescaled to {a.rate}% CAV (from {a.src})")
