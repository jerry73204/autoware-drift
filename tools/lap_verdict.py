#!/usr/bin/env python3
"""One line saying how a lap ended: CRASH, PASS or FAIL, before anyone digs into it.

    tools/lap_verdict.py <scenario output_directory> [--since <unix time>] [--contacts <dir>]
        [--min-impulse 2000]

--since is the lap's start: the contact reports read are those written from then until five
minutes after SSv2's junit (the ego's report is written when its run ends), so a lap never
reads another lap's report.

A crash is a contact in the bridge's contact report (scenario-world-bridge, roadmap 021:
$ROS_HOME/swb/contacts, one JSON per ego run) whose impulse passes --min-impulse (N s; a
kerb's brush is a few hundred, lap 19's barrier 19 000), or SSv2 ending the scenario on the
ego's acceleration out of its catalogue's range, which a hit is. Otherwise the junit
verdict: PASS with no failure or error, FAIL with SSv2's message.

Exits 0 on PASS, 1 on FAIL, 2 on CRASH.
"""
import argparse
import glob
import json
import os
import re
import sys
import time
import xml.etree.ElementTree as ET


def junit(out_dir):
    files = glob.glob(os.path.join(out_dir, "**", "*.junit.xml"), recursive=True)
    if not files:
        return None, "no junit result (the runner did not finish)", None
    path = max(files, key=os.path.getmtime)
    root = ET.parse(path).getroot()
    for case in root.iter("testcase"):
        for tag in ("failure", "error"):
            el = case.find(tag)
            if el is not None:
                return False, el.get("message") or (el.text or "").strip(), os.path.getmtime(path)
    return True, "", os.path.getmtime(path)


def contacts(directory, since, until):
    reports = []
    for path in sorted(glob.glob(os.path.join(directory, "*.json"))):
        if not since <= os.path.getmtime(path) <= until:
            continue
        try:
            reports.append(json.load(open(path)))
        except (OSError, ValueError):
            continue
    return [c for r in reports for c in r.get("contacts", [])]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out_dir")
    ap.add_argument("--since", type=float, default=0.0, help="unix time: reports older are skipped")
    ap.add_argument("--contacts", default=os.path.join(os.environ.get("ROS_HOME", os.path.expanduser("~/.ros")),
                                                       "swb", "contacts"))
    ap.add_argument("--min-impulse", type=float, default=2000.0)
    a = ap.parse_args()

    ok, message, ended = junit(a.out_dir)
    until = (ended or time.time()) + 300
    hits = [c for c in contacts(a.contacts, a.since, until) if (c.get("max_impulse") or 0) >= a.min_impulse]
    accel = re.search(r"current acceleration \(which is (-?[0-9.]+)\) is out of range", message or "")
    if hits:
        c = max(hits, key=lambda c: c["max_impulse"])
        what = c.get("name") or c.get("type") or "?"
        print(f"CRASH: hit {what} at sim time {c.get('first_sim_time', 0) - 1000:.2f} s "
              f"(impulse {c['max_impulse']:.0f} N s{', ' + str(len(hits)) + ' contacts' if len(hits) > 1 else ''})")
        return 2
    if accel:
        print(f"CRASH: SSv2 ended it on the ego's acceleration {accel.group(1)} m/s^2 (a hit)")
        return 2
    if ok:
        print("PASS")
        return 0
    print(f"FAIL: {message}")
    return 1


if __name__ == "__main__":
    sys.exit(main())
