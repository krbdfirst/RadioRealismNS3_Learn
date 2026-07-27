#!/usr/bin/env python3
"""
make_cav_ns3_inputs.py

Turn a SUMO FCD CSV (from PropagationRealismProject/run_sumo_fcd_for_ns3.sh)
into NS-3 inputs for the NR V2X sidelink replay, keeping ONLY the CAVs
(vehicle_type == veh_av) — exactly the set that carries a network stack in the
OMNeT/Veins setup (veh_human run pure SUMO with no radio, so they are excluded
here too; OMNeT also uses IdealObstacleLoss, so human cars do not block the
channel and are correctly absent as NS-3 nodes).

Outputs (in --outdir):
  <tag>_mobility.tcl   ns-2 mobility (Ns2MobilityHelper), antenna height = --height
  <tag>_meta.csv       nodeId,vehId,departTime,arriveTime   (per-CAV lifetime)
  <tag>_info.txt       key=value: numUes, height, tBegin, tEnd

Node ids are 0..K-1 assigned in order of first appearance (depart time, then id),
so they are stable and consistent between the .tcl and the meta file.

Usage:
  python3 make_cav_ns3_inputs.py --fcd-csv fcd_cav25.csv --tag cav25 --outdir ./inputs
  (options: --height 1.5  --begin 0  --end 200)
"""
import argparse
import csv
import math
import os
import sys


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fcd-csv", required=True, help="SUMO FCD CSV (xml2csv output)")
    ap.add_argument("--tag", required=True, help="output basename, e.g. cav25")
    ap.add_argument("--outdir", default=".", help="output directory")
    ap.add_argument("--height", type=float, default=1.5, help="antenna z height (m)")
    ap.add_argument("--cav-type", default="veh_av", help="SUMO type treated as CAV")
    ap.add_argument("--begin", type=float, default=0.0, help="clip trace begin (s)")
    ap.add_argument("--end", type=float, default=1e12, help="clip trace end (s)")
    ap.add_argument("--jitter", type=float, default=0.0,
                    help="unique per-node position offset step (m); >0 guarantees no two "
                         "nodes ever share a (rounded) coordinate -> avoids distance-0 SINR NaN")
    args = ap.parse_args()

    os.makedirs(args.outdir, exist_ok=True)

    # Read FCD CSV, keep only CAV rows: (t, vid, x, y, speed)
    # xml2csv flattens to columns: timestep_time, vehicle_id, vehicle_x, vehicle_y,
    # vehicle_speed, vehicle_type, ...   (and person_* columns which we ignore)
    tracks = {}  # vid -> list of (t, x, y, speed)
    with open(args.fcd_csv, newline="") as f:
        rd = csv.DictReader(f)
        # tolerate both 'vehicle_*' and bare names
        def col(row, *names):
            for n in names:
                if n in row and row[n] not in ("", None):
                    return row[n]
            return None
        for row in rd:
            vtype = col(row, "vehicle_type", "type")
            if vtype != args.cav_type:
                continue
            vid = col(row, "vehicle_id", "id")
            t = col(row, "timestep_time", "time")
            x = col(row, "vehicle_x", "x")
            y = col(row, "vehicle_y", "y")
            sp = col(row, "vehicle_speed", "speed")
            if None in (vid, t, x, y):
                continue
            t = float(t)
            if t < args.begin or t > args.end:
                continue
            tracks.setdefault(vid, []).append(
                (t, float(x), float(y), float(sp) if sp is not None else 0.0)
            )

    if not tracks:
        sys.exit(f"ERROR: no rows with vehicle_type=={args.cav_type} in {args.fcd_csv}")

    # Stable node-id order: by depart time, then vehicle id
    for vid in tracks:
        tracks[vid].sort(key=lambda r: r[0])
    order = sorted(tracks.keys(), key=lambda v: (tracks[v][0][0], v))

    mob_path = os.path.join(args.outdir, f"{args.tag}_mobility.tcl")
    meta_path = os.path.join(args.outdir, f"{args.tag}_meta.csv")
    info_path = os.path.join(args.outdir, f"{args.tag}_info.txt")

    t_min = min(tr[0][0] for tr in tracks.values())
    t_max = max(tr[-1][0] for tr in tracks.values())

    with open(mob_path, "w") as mob, open(meta_path, "w") as meta:
        meta.write("nodeId,vehId,departTime,arriveTime\n")
        for nid, vid in enumerate(order):
            pts = tracks[vid]
            # unique, deterministic per-node offset (2D grid of multiples of --jitter)
            # so two nodes can never collapse to the same rounded coord -> no SINR NaN.
            ox = args.jitter * ((nid % 20) + 1)
            oy = args.jitter * ((nid // 20) + 1)
            x0, y0 = pts[0][1] + ox, pts[0][2] + oy
            depart, arrive = pts[0][0], pts[-1][0]
            # initial position + fixed antenna height
            mob.write(f"$node_({nid}) set X_ {x0:.2f}\n")
            mob.write(f"$node_({nid}) set Y_ {y0:.2f}\n")
            mob.write(f"$node_({nid}) set Z_ {args.height:.2f}\n")
            # one setdest per interval: travel p_k -> p_{k+1}, arriving at t_{k+1}
            for k in range(len(pts) - 1):
                t_k, x_k, y_k, _ = pts[k]
                t_n, x_n, y_n, _ = pts[k + 1]
                dt = t_n - t_k
                if dt <= 0:
                    continue
                dist = math.hypot(x_n - x_k, y_n - y_k)
                spd = dist / dt  # m/s to arrive exactly at next sample
                mob.write(
                    f'$ns_ at {t_k:.2f} "$node_({nid}) setdest {x_n + ox:.2f} {y_n + oy:.2f} {spd:.4f}"\n'
                )
            # stop motion at the last sample
            xl, yl = pts[-1][1] + ox, pts[-1][2] + oy
            mob.write(f'$ns_ at {arrive:.2f} "$node_({nid}) setdest {xl:.2f} {yl:.2f} 0.0"\n')
            meta.write(f"{nid},{vid},{depart:.2f},{arrive:.2f}\n")

    with open(info_path, "w") as info:
        info.write(f"numUes={len(order)}\n")
        info.write(f"height={args.height}\n")
        info.write(f"tBegin={t_min:.2f}\n")
        info.write(f"tEnd={t_max:.2f}\n")

    print(f"[{args.tag}] CAV UEs={len(order)}  trace=[{t_min:.1f},{t_max:.1f}]s")
    print(f"  {mob_path}")
    print(f"  {meta_path}")
    print(f"  {info_path}")


if __name__ == "__main__":
    main()
