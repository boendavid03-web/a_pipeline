#!/usr/bin/env python3
"""Read-only, identical scan/TF/rolling-grid audit of the two frozen ROS bags.

Run after `source /opt/ros/humble/setup.bash` with /usr/bin/python3. No ROS
nodes are started and no bag or runtime parameters are modified.
"""

import bisect
import json
import math
import sqlite3
import statistics
from collections import Counter, defaultdict
from pathlib import Path
import yaml

from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock
try:
    from hunav_msgs.msg import Agents
except ImportError:
    Agents = None


ROOT = Path(__file__).resolve().parent
BAGS = {
    "short": ROOT / "isaac_sim/backends/isaac5/generated/frozen_frequency_control_20261001/1hz/raw_rosbag/raw_rosbag_0.db3",
    "default": ROOT / "isaac_sim/backends/isaac5/generated/frozen_default_hunav_20261001/raw_rosbag/raw_rosbag_0.db3",
}
OUT = ROOT / "ARENA_LOCAL_COSTMAP_AUDIT_20261001.json"
WALLS_PATH = Path("/home/user/arena_isaac5_host_overlay_ws/src/arena_simulation_setup/worlds/map_empty/map/walls.yaml")
WALLS = yaml.safe_load(WALLS_PATH.read_text())["walls"]


def stamp(x):
    return x.sec + x.nanosec * 1e-9


def quat_mul(a, b):
    x, y, z, w = a
    X, Y, Z, W = b
    return (w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
            w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z)


def quat_rotate(q, p):
    x, y, z, w = q
    u = (x, y, z)
    uv = (u[1]*p[2]-u[2]*p[1], u[2]*p[0]-u[0]*p[2], u[0]*p[1]-u[1]*p[0])
    uuv = (u[1]*uv[2]-u[2]*uv[1], u[2]*uv[0]-u[0]*uv[2], u[0]*uv[1]-u[1]*uv[0])
    return tuple(p[i]+2*(w*uv[i]+uuv[i]) for i in range(3))


def q_slerp(a, b, f):
    dot = sum(x*y for x, y in zip(a, b))
    if dot < 0:
        b = tuple(-v for v in b)
        dot = -dot
    if dot > .9995:
        q = tuple(x + f*(y-x) for x, y in zip(a, b))
        n = math.sqrt(sum(x*x for x in q))
        return tuple(x/n for x in q)
    omega = math.acos(max(-1, min(1, dot)))
    den = math.sin(omega)
    return tuple((math.sin((1-f)*omega)*x + math.sin(f*omega)*y)/den
                 for x, y in zip(a, b))


def compose(a, b):
    v = quat_rotate(a[1], b[0])
    return (tuple(a[0][i]+v[i] for i in range(3)), quat_mul(a[1], b[1]))


def apply(tf, p):
    v = quat_rotate(tf[1], p)
    return tuple(tf[0][i]+v[i] for i in range(3))


def load_bag(path):
    conn = sqlite3.connect(f"file:{path}?mode=ro", uri=True)
    topics = {r[0]: r[1] for r in conn.execute("select id,name from topics")}
    data = defaultdict(list)
    msg_types = {"/tf": TFMessage, "/tf_static": TFMessage,
                 "/task_generator_node/jackal/lidar": LaserScan,
                 "/task_generator_node/jackal/local_costmap/costmap": OccupancyGrid,
                 "/task_generator_node/jackal/odom": Odometry,
                 "/task_generator_node/map": OccupancyGrid,
                 "/clock": Clock}
    if Agents is not None:
        msg_types["/task_generator_node/human_states"] = Agents
    for topic_id, receipt, raw in conn.execute("select topic_id,timestamp,data from messages order by timestamp"):
        name = topics[topic_id]
        if name in msg_types:
            data[name].append((receipt * 1e-9, deserialize_message(raw, msg_types[name])))
    conn.close()
    return data


def transform_index(data):
    edges = defaultdict(list)
    static = defaultdict(list)
    for topic, target in (("/tf", edges), ("/tf_static", static)):
        for receipt, msg in data[topic]:
            for t in msg.transforms:
                key = (t.header.frame_id.lstrip("/"), t.child_frame_id.lstrip("/"))
                v = t.transform.translation
                q = t.transform.rotation
                target[key].append((stamp(t.header.stamp), receipt,
                                    ((v.x, v.y, v.z), (q.x, q.y, q.z, q.w))))
    for values in edges.values():
        values.sort(key=lambda x: x[0])
    return edges, static


def lookup(edges, static, key, t):
    # Strict recorded-time interpolation. For a dynamic edge, /tf_static must
    # not conceal a recorded extrapolation gap or duplicate edge.
    values = edges.get(key)
    if not values:
        if key in static:
            return static[key][-1][2], 0.0, "static"
        return None, None, "missing"
    times = [r[0] for r in values]
    pos = bisect.bisect_right(times, t)
    if pos == 0:
        return None, None, "before_first"
    a = values[pos-1]
    if t == a[0]:
        return a[2], 0.0, "exact"
    if pos == len(values):
        return None, t-a[0], "after_last"
    b = values[pos]
    f = (t-a[0])/(b[0]-a[0]) if b[0] > a[0] else 0.0
    xyz = tuple(a[2][0][i] + f*(b[2][0][i]-a[2][0][i]) for i in range(3))
    return (xyz, q_slerp(a[2][1], b[2][1], f)), t-a[0], "interpolated"


def cell(grid, x, y):
    ox, oy = grid.info.origin.position.x, grid.info.origin.position.y
    res = grid.info.resolution
    i, j = math.floor((x-ox)/res), math.floor((y-oy)/res)
    if not (0 <= i < grid.info.width and 0 <= j < grid.info.height):
        return None, (i, j)
    return int(grid.data[j*grid.info.width+i]), (i, j)


def wall_distance(x, y):
    best = float("inf")
    for a, b in WALLS:
        dx, dy = b[0]-a[0], b[1]-a[1]
        f = max(0.0, min(1.0, ((x-a[0])*dx+(y-a[1])*dy)/(dx*dx+dy*dy)))
        best = min(best, math.hypot(x-a[0]-f*dx, y-a[1]-f*dy))
    return best


def summary(name):
    data = load_bag(BAGS[name])
    scans = data["/task_generator_node/jackal/lidar"]
    grids = data["/task_generator_node/jackal/local_costmap/costmap"]
    odoms = data["/task_generator_node/jackal/odom"]
    map_grid = data["/task_generator_node/map"][0][1] if data["/task_generator_node/map"] else None
    edges, static = transform_index(data)
    clock_receipts = [r for r,_ in data["/clock"]]
    clock_stamps = [stamp(c.clock) for _,c in data["/clock"]]
    people = data["/task_generator_node/human_states"]
    people_receipts = [r for r,_ in people]
    # A scan can only be consumed after the relevant TF has arrived. Compare
    # TF receipt time with scan receipt time, independently of offline TF
    # interpolation, and report the first future TF arrival for each edge.
    causal_edges = {}
    for key, values in edges.items():
        causal_edges[key] = sorted([(receipt, ts) for ts, receipt, _ in values])
    chain = [("map", "jackal/odom"), ("jackal/odom", "jackal/base_link"),
             ("jackal/base_link", "jackal/chassis_link"),
             ("jackal/chassis_link", "jackal/lidar_link")]
    last_by_edge = {}
    tf_backward_jumps = Counter()
    invalid_parent = Counter()
    for _, msg in data["/tf"]:
        for tr in msg.transforms:
            key = (tr.header.frame_id, tr.child_frame_id)
            if not tr.header.frame_id:
                invalid_parent[tr.child_frame_id] += 1
            ts = stamp(tr.header.stamp)
            if key in last_by_edge and ts < last_by_edge[key]-1e-6:
                tf_backward_jumps[str(key)] += 1
            last_by_edge[key] = ts
    gt = [stamp(g.header.stamp) for _, g in grids]
    finite_total = near_total = markable = valid_scans = near_scans = 0
    markable_scans = 0
    tf_fail = Counter()
    tf_ages = defaultdict(list)
    tf_valid = 0
    causal_ready = 0
    causal_delay = []
    scan_clock_lag = []
    scans_clock_before_tf = 0
    sample_rows = []
    all_rows = []
    near_scan_ids = []
    near_value_hist = Counter()
    near_z = []
    markable_actual = Counter()
    markable_map = Counter()
    map_near_occ = 0
    geometric_wall_hits = 0
    grid_after = 0
    scan_fields = Counter()
    per_scan = []
    for si, (receipt, scan) in enumerate(scans):
        t = stamp(scan.header.stamp)
        ci = bisect.bisect_right(clock_receipts, receipt)-1
        if ci >= 0:
            scan_clock_lag.append(clock_stamps[ci]-t)
        future_receipts = []
        for pair in chain:
            candidates = [r for r, ts in causal_edges.get(pair, []) if ts >= t + .3]
            if candidates:
                future_receipts.append(min(candidates))
        if len(future_receipts) == len(chain):
            ready_at = max(future_receipts)
            causal_delay.append(ready_at-receipt)
            causal_ready += ready_at <= receipt
            scans_clock_before_tf += ready_at > receipt
        else:
            ready_at = None
        scan_fields[(scan.header.frame_id, len(scan.ranges), scan.angle_min,
                     scan.angle_max, scan.angle_increment, scan.range_min,
                     scan.range_max, scan.scan_time, scan.time_increment)] += 1
        ranges = scan.ranges
        finite = [(i, float(v)) for i,v in enumerate(ranges)
                  if math.isfinite(v) and scan.range_min <= v <= scan.range_max]
        near = [(i,v) for i,v in finite if v <= 2.5]
        finite_total += len(finite)
        near_total += len(near)
        valid_scans += bool(finite)
        near_scans += bool(near)
        if near:
            near_scan_ids.append(si)
        transforms = []
        ages = []
        for pair in chain:
            tr, age, reason = lookup(edges, static, pair, t)
            if tr is None:
                tf_fail[(pair, reason)] += 1
            else:
                transforms.append(tr)
                ages.append(age)
                tf_ages[str(pair)].append(age)
        scan_row = {"scan_index":si, "header_s":round(t,6),
                    "receipt_s":round(receipt,6), "near_returns":len(near),
                    "tf_available_offline":len(transforms)==len(chain),
                    "tf_age_max_s":round(max(ages),3) if ages else None,
                    "tf_future_available_after_receipt_wall_s":round(ready_at-receipt,3) if ready_at else None,
                    "geometric_candidate_returns":None}
        per_scan.append(scan_row)
        if len(transforms) != len(chain):
            continue
        tf_valid += 1
        map_lidar = transforms[0]
        for tr in transforms[1:]:
            map_lidar = compose(map_lidar, tr)
        odom_lidar = compose(compose(transforms[1], transforms[2]), transforms[3])
        robot_map = compose(transforms[0], transforms[1])[0]
        robot_odom = transforms[1][0]
        gidx = bisect.bisect_left(gt, t)
        grid = grids[gidx][1] if gidx < len(grids) and gt[gidx]-t <= .5 else None
        if grid:
            grid_after += 1
        local_markable = 0
        best = None
        for beam, rng in near:
            angle = scan.angle_min + beam * scan.angle_increment
            p_lidar = (rng*math.cos(angle), rng*math.sin(angle), 0.0)
            p_odom = apply(odom_lidar, p_lidar)
            p_map = apply(map_lidar, p_lidar)
            near_z.append(p_odom[2])
            robot_dist = math.hypot(p_odom[0]-robot_odom[0], p_odom[1]-robot_odom[1])
            actual, ij = cell(grid, p_odom[0], p_odom[1]) if grid else (None, None)
            static_value, _ = cell(map_grid, p_map[0], p_map[1]) if map_grid else (None, None)
            wall_dist = wall_distance(p_map[0], p_map[1])
            nearest_human = None
            if people:
                hi = bisect.bisect_left(people_receipts, receipt)
                hi = min(max(hi, 0), len(people)-1)
                if hi and abs(people_receipts[hi-1]-receipt) < abs(people_receipts[hi]-receipt):
                    hi -= 1
                human_dists = [(math.hypot(p_map[0]-a.position.position.x,
                                           p_map[1]-a.position.position.y), a.name)
                               for a in people[hi][1].agents]
                if human_dists:
                    dist, human_name = min(human_dists)
                    nearest_human = [human_name, round(dist,3),
                                     round(people_receipts[hi]-receipt,3)]
            height_ok = 0 <= p_odom[2] < .8  # 16 * 0.05 m voxel span
            footprint_ok = robot_dist > .2
            in_grid = actual is not None
            candidate = height_ok and footprint_ok and in_grid
            near_value_hist[(height_ok, footprint_ok, in_grid)] += 1
            if candidate:
                local_markable += 1
                markable += 1
                geometric_wall_hits += wall_dist <= .15
                markable_actual[actual] += 1
                markable_map[static_value] += 1
            if static_value is not None and static_value >= 50:
                map_near_occ += 1
            row = {
                "scan_index": si, "scan_header_s": round(t, 6),
                "scan_receipt_s": round(receipt, 6), "beam": beam,
                "angle_rad": round(angle, 5), "range_m": round(rng, 4),
                "robot_map_xyz": [round(v,3) for v in robot_map],
                "robot_odom_xyz": [round(v,3) for v in robot_odom],
                "lidar_map_xyz": [round(v,3) for v in map_lidar[0]],
                "lidar_odom_xyz": [round(v,3) for v in odom_lidar[0]],
                "hit_map_xyz": [round(v,3) for v in p_map],
                "hit_odom_xyz": [round(v,3) for v in p_odom],
                "robot_distance_m": round(robot_dist,3),
                "grid_stamp_s": round(gt[gidx],6) if grid else None,
                "grid_bounds_odom": [round(grid.info.origin.position.x,3),round(grid.info.origin.position.y,3),
                                     round(grid.info.origin.position.x+grid.info.width*grid.info.resolution,3),
                                     round(grid.info.origin.position.y+grid.info.height*grid.info.resolution,3)] if grid else None,
                "grid_cell": ij, "actual_cell": actual, "static_map_cell": static_value,
                "source_wall_distance_m": round(wall_dist,3),
                "nearest_human_name_distance_m_receipt_delta_s": nearest_human,
                "height_ok": height_ok, "footprint_ok": footprint_ok,
                "geometric_candidate": candidate,
                "tf_age_max_s": round(max(ages),3),
            }
            if best is None or (candidate, -rng) > (best["geometric_candidate"], -best["range_m"]):
                best = row
        if best:
            all_rows.append(best)
        scan_row["geometric_candidate_returns"] = local_markable
        if local_markable:
            markable_scans += 1
    # Ten evenly distributed valid-return timestamps, with extra examples of
    # any markable geometry rather than selectively choosing favorable scans.
    if all_rows:
        picks = sorted(set(round(i*(len(all_rows)-1)/9) for i in range(10)))
        sample_rows = [all_rows[i] for i in picks]
    scan_times = [stamp(s.header.stamp) for _,s in scans]
    scan_receipts = [r for r,_ in scans]
    def intervals(vals):
        gaps = [b-a for a,b in zip(vals, vals[1:])]
        return {"min": min(gaps), "median": statistics.median(gaps),
                "p95": sorted(gaps)[int(.95*(len(gaps)-1))], "max": max(gaps)} if gaps else None
    pair_data = {}
    for key in set(edges)|set(static):
        vals = edges.get(key, [])
        pair_data[str(key)] = {
            "dynamic_count": len(vals), "static_count": len(static.get(key, [])),
            "dynamic_stamp_range": [vals[0][0], vals[-1][0]] if vals else None,
            "dynamic_intervals": intervals([x[0] for x in vals]) if len(vals)>1 else None,
            "static_stamp": static[key][-1][0] if key in static else None,
        }
    grid_occ = [sum(v>0 for v in g.data) for _,g in grids]
    result = {
        "bag": str(BAGS[name]), "scan_count": len(scans),
        "scan_fields": [{"frame": k[0], "beams": k[1], "angle_min": k[2],
                         "angle_max": k[3], "angle_increment": k[4],
                         "range_min": k[5], "range_max": k[6],
                         "scan_time": k[7], "time_increment": k[8], "count": v}
                        for k,v in scan_fields.items()],
        "scan_header_range_s": [min(scan_times), max(scan_times)],
        "scan_receipt_range_s": [min(scan_receipts), max(scan_receipts)],
        "scan_header_intervals_s": intervals(scan_times),
        "scan_receipt_intervals_s": intervals(scan_receipts),
        "finite_returns": finite_total, "near_returns": near_total,
        "scans_with_near": near_scans, "scans_with_finite": valid_scans,
        "tf_valid_scans": tf_valid, "tf_valid_ratio": tf_valid/len(scans),
        "tf_causally_ready_at_scan_receipt": causal_ready,
        "tf_future_arrival_delay_wall_s": {"median":statistics.median(causal_delay),
                                           "p95":sorted(causal_delay)[int(.95*(len(causal_delay)-1))],
                                           "max":max(causal_delay)} if causal_delay else None,
        "scan_header_vs_latest_clock_at_receipt_s": {"median":statistics.median(scan_clock_lag),
                                                       "min":min(scan_clock_lag),
                                                       "max":max(scan_clock_lag)} if scan_clock_lag else None,
        "tf_failure_reasons": {str(k):v for k,v in tf_fail.items()},
        "tf_age_s": {k:{"median":statistics.median(v), "max":max(v)} for k,v in tf_ages.items()},
        "tf_pairs": pair_data,
        "tf_backward_stamp_jumps_in_receipt_order": dict(tf_backward_jumps),
        "tf_empty_parent_transforms": dict(invalid_parent),
        "geometric_markable_near_returns": markable,
        "geometric_markable_near_wall_returns": geometric_wall_hits,
        "scans_with_geometric_markable": markable_scans,
        "near_geometry_breakdown": {str(k):v for k,v in near_value_hist.items()},
        "near_hit_z_odom_min_max": [min(near_z),max(near_z)] if near_z else None,
        "geometric_markable_actual_cell_values": dict(markable_actual),
        "geometric_markable_static_map_values": dict(markable_map),
        "near_hits_on_static_occupied_map_cells": map_near_occ,
        "scans_with_following_grid_0p5s": grid_after,
        "local_grids": len(grids), "max_occupied_cells": max(grid_occ) if grid_occ else None,
        "grids_with_occupied_cells": sum(v>0 for v in grid_occ),
        "sample_rows": sample_rows,
        "per_scan": per_scan,
    }
    return result


if __name__ == "__main__":
    results = {name: summary(name) for name in BAGS}
    def sanitize(x):
        if isinstance(x, float) and not math.isfinite(x):
            return str(x)
        if isinstance(x, dict):
            return {k:sanitize(v) for k,v in x.items()}
        if isinstance(x, list):
            return [sanitize(v) for v in x]
        return x
    OUT.write_text(json.dumps(sanitize(results), indent=2, allow_nan=False) + "\n")
    for name, r in results.items():
        print(name, {k:r[k] for k in ("scan_count","near_returns","tf_valid_ratio",
                                         "geometric_markable_near_returns","local_grids",
                                         "max_occupied_cells")})
    print(OUT)
