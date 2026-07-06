#!/usr/bin/env python3
"""Measure inter-camera capture-timestamp spread in an MCAP recording.

For each frame of the reference camera it finds the nearest-in-time frame in
every other camera and reports the per-shot spread (max - min). With perfect
software sync the spread collapses to the PTP residual (tens of microseconds);
free-running cameras spread out to roughly half a frame period.

It reads each image's header.stamp (the device PTP capture time), NOT the MCAP
log time, so the metric reflects when photons were captured, not when bytes
arrived - bandwidth/transfer delays do not affect it.

Usage:
    analyze_sync.py synced.mcap [baseline.mcap] --topics /cam0/images /cam1/images /cam2/images

Run with no arguments to execute the built-in self-check.
"""
import argparse
import bisect
import statistics
import sys


def _percentile(sorted_vals, pct):
    if not sorted_vals:
        return float("nan")
    k = (len(sorted_vals) - 1) * pct / 100.0
    lo = int(k)
    hi = min(lo + 1, len(sorted_vals) - 1)
    return sorted_vals[lo] + (sorted_vals[hi] - sorted_vals[lo]) * (k - lo)


def nearest(sorted_list, x):
    """Value in sorted_list closest to x."""
    i = bisect.bisect_left(sorted_list, x)
    if i == 0:
        return sorted_list[0]
    if i == len(sorted_list):
        return sorted_list[-1]
    before, after = sorted_list[i - 1], sorted_list[i]
    return after if (after - x) < (x - before) else before


def shot_spreads_ns(stamps_by_topic):
    """Per-shot spread (ns): for each reference-topic frame, pair the nearest
    frame from every other topic and take max - min across the matched set."""
    topics = list(stamps_by_topic)
    ref = sorted(stamps_by_topic[topics[0]])
    others = [sorted(stamps_by_topic[t]) for t in topics[1:]]
    others = [o for o in others if o]
    spreads = []
    for t in ref:
        matched = [t] + [nearest(o, t) for o in others]
        spreads.append(max(matched) - min(matched))
    return spreads


def summarize(spreads_ns):
    us = sorted(s / 1000.0 for s in spreads_ns)  # ns -> microseconds
    return {
        "shots": len(us),
        "mean_us": statistics.mean(us) if us else float("nan"),
        "std_us": statistics.pstdev(us) if len(us) > 1 else 0.0,
        "min_us": us[0] if us else float("nan"),
        "median_us": statistics.median(us) if us else float("nan"),
        "p95_us": _percentile(us, 95),
        "p99_us": _percentile(us, 99),
        "max_us": us[-1] if us else float("nan"),
    }


def read_stamps(mcap_path, topics):
    """Pull header.stamp (device PTP time, ns) for each topic from an MCAP."""
    import mcap_unpacker as mu

    unpacker = mu.McapUnpacker(mcap_path)
    stamps = {t: [] for t in topics}
    for msg in unpacker.messages(set(topics)):
        stamps[msg.topic].append(msg.payload.header.stamp.to_nanoseconds())
    return stamps


def report(label, stamps_by_topic):
    counts = {t: len(v) for t, v in stamps_by_topic.items()}
    s = summarize(shot_spreads_ns(stamps_by_topic))
    print(f"\n== {label} ==")
    print(f"  frames per topic: {counts}")
    print(f"  shots: {s['shots']}")
    print(f"  spread (us)  mean={s['mean_us']:.1f}  std={s['std_us']:.1f}  "
          f"median={s['median_us']:.1f}  p95={s['p95_us']:.1f}  "
          f"p99={s['p99_us']:.1f}  max={s['max_us']:.1f}")
    return s


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("synced_mcap")
    ap.add_argument("baseline_mcap", nargs="?")
    ap.add_argument("--topics", nargs="+", required=True,
                    help="image topics, one per camera (first is the reference)")
    args = ap.parse_args(argv)

    synced = report("synced", read_stamps(args.synced_mcap, args.topics))
    if args.baseline_mcap:
        base = report("baseline (no sync)",
                      read_stamps(args.baseline_mcap, args.topics))
        if synced["mean_us"] > 0:
            print(f"\nmean spread improved {base['mean_us'] / synced['mean_us']:.1f}x "
                  f"({base['mean_us']:.1f}us -> {synced['mean_us']:.1f}us)")

    ok = synced["mean_us"] < 1000.0  # 1 ms target
    print(f"\n{'PASS' if ok else 'FAIL'}: synced mean spread "
          f"{synced['mean_us']:.1f}us {'<' if ok else '>='} 1000us target")
    return 0 if ok else 1


def _selfcheck():
    import random
    random.seed(0)
    n, period = 300, 100_000_000  # 10 Hz
    base_t = [i * period for i in range(n)]
    # synced: every camera fires together within <1us jitter.
    synced = {f"/cam{c}/images": [t + random.randint(-500, 500) for t in base_t]
              for c in range(3)}
    # baseline: each camera free-runs at the same rate but an independent phase.
    offsets = [0, period // 3, 2 * period // 3]
    base = {f"/cam{c}/images": [t + offsets[c] + random.randint(-500, 500)
                                for t in base_t] for c in range(3)}
    s = summarize(shot_spreads_ns(synced))
    b = summarize(shot_spreads_ns(base))
    assert s["mean_us"] < 5, s
    assert b["mean_us"] > 1000, b
    assert b["mean_us"] > 100 * s["mean_us"], (s, b)
    print(f"selfcheck OK: synced mean {s['mean_us']:.2f}us, "
          f"baseline mean {b['mean_us']:.1f}us")


if __name__ == "__main__":
    if len(sys.argv) == 1:
        _selfcheck()
    else:
        sys.exit(main(sys.argv[1:]))
