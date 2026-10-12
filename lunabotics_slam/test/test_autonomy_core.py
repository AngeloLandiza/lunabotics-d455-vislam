"""Offline checks of scripts/autonomy_core.py against the sim tests it comes from (no ROS needed).

  python3 lunabotics_slam/test/test_autonomy_core.py [tags|detector|filter|planner ...]

tags      AprilTag images drawn into a pinhole camera at known robot poses; detection + solve must recover the pose
detector  sandbox L1 frames along a straight drive: HazardMap fed point clouds (ROS form) vs detect.py fed the
          sandbox's own ray-marched scans (approach.py loop); same obstacles found
filter    drift.py drive and L1 frames replayed through WallFilter message by message; error at each leg end
planner   mission legs in a sandbox world driven by Leg on the skid-steer model with slip; hazards revealed late
          trigger a replan
closed    everything together with no true pose anywhere: L1 frames from the true pose feed the WallFilter and the
          HazardMap (placed with the estimated pose), and MissionRunner drives on the estimate and the detected hazards
"""
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'scripts'))
import autonomy_core as ac           # noqa: E402


def test_tags(n=40, seed=0):
    import cv2
    rng = np.random.default_rng(seed)
    fix = ac.TagFix()
    ts = ac.tag_start
    ts.set_hfov(2.0)                 # the wide rear lens arena.sh mounts (2.0 rad, the Isaac two-tag check) ...
    ts.set_pitch(-15.0)              # ... pitched 15 deg down, as in arena.sh
    K = ts.K.copy()
    W, H = ts.W_PX, ts.H_PX
    size_px = 400
    dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    T_bo = np.eye(4)
    T_bo[:3, :3], T_bo[:3, 3] = ts.R_BASE_OPT, ts.T_BASE_OPT
    errs, seen = [], 0
    for _ in range(n):
        # start zone, rear camera turned towards one tag or the corner between them (Isaac check: turn until seen)
        x, y = rng.uniform(0.7, 1.7), rng.uniform(0.7, 1.7)
        (ax, ay, _), (bx, by, _) = ac.val('tag_xy')
        tx, ty = [(ax, ay), (bx, by), (bx, ay)][rng.integers(3)]
        yaw = math.atan2(y - ty, x - tx) + rng.normal(0, math.radians(10))
        R_wc, t_wc = ts.cam_pose((x, y, yaw))
        img = np.full((H, W), 140, np.uint8)
        for tid, Wc in fix.world.items():
            pc = (Wc - t_wc) @ R_wc
            if np.any(pc[:, 2] < 0.1):
                continue
            uv = (pc[:, :2] / pc[:, 2:]) @ K[:2, :2].T + K[:2, 2]
            # the 8x8-cell marker image (6x6 data + black border) inside a 1-cell white border: black square = tag_size
            mk = cv2.aruco.generateImageMarker(dic, tid, size_px)
            pad = size_px // 8
            full = cv2.copyMakeBorder(mk, pad, pad, pad, pad, cv2.BORDER_CONSTANT, value=255)
            src = np.float32([[pad, pad], [pad + size_px, pad], [pad + size_px, pad + size_px], [pad, pad + size_px]])
            Hm = cv2.getPerspectiveTransform(src, uv.astype(np.float32))
            warped = cv2.warpPerspective(full, Hm, (W, H), flags=cv2.INTER_LINEAR, borderValue=0)
            mask = cv2.warpPerspective(np.full_like(full, 255), Hm, (W, H), flags=cv2.INTER_NEAREST, borderValue=0)
            img[mask > 0] = warped[mask > 0]
        img = np.clip(img + rng.normal(0, 3, img.shape), 0, 255).astype(np.uint8)
        found = fix.detect(img)
        if not found:
            continue
        seen += 1
        est = fix.solve(found, K, None, T_bo)
        errs.append((math.hypot(est[0] - x, est[1] - y), abs(math.degrees(ac.wrap(est[2] - yaw))), est[3]))
    e = np.array(errs)
    print(f'tags: {seen}/{n} images with a tag; position error median {np.median(e[:, 0]) * 100:.2f} cm, '
          f'max {e[:, 0].max() * 100:.2f} cm; heading median {np.median(e[:, 1]):.2f} deg, max {e[:, 1].max():.2f}; '
          f'two-tag fixes {int((e[:, 2] == 2).sum())}')
    assert seen >= n // 3 and np.median(e[:, 0]) < 0.03 and e[:, 1].max() < 3.0
    return e


def test_detector(seeds=(3000, 3001, 3002, 3003), ds=0.1):
    import params as pr
    from arena import Arena
    from sensors import Lidar
    import selfmask
    import localize as lz
    det = ac.detect
    xyz, R, table = lz.parts('front45')
    rows = []
    for seed in seeds:
        rng = np.random.default_rng(seed)
        arena = Arena(rng)
        lid = Lidar('l1', mount_xyz=xyz, mount_R=R, body=lz._CACHE['body'])
        x0, y0 = 0.6, rng.uniform(1.0, 4.0)
        hm = ac.HazardMap(size=(arena.L, arena.W))
        grid = det.Grid(arena.L, arena.W, ac.v(pr.DETECT, 'cell'))
        L = det.new_layers(grid)
        lp, ln = np.zeros((grid.nx, grid.ny)), np.zeros((grid.nx, grid.ny))
        t0 = time.time()
        t_core = 0.0
        n = int((arena.L - 1.6 - x0) / ds)
        for i in range(n):
            pose = (x0 + i * ds, y0, 0.0)
            s = lid.scan(arena, pose, rng)
            s, _ = selfmask.apply(s, pose, table=table)
            # reference: the sandbox scan as approach.py uses it
            det.add_scan(L, s, grid)
            g = det.fit_ground(L, grid)
            dp, dn = det.logodds_delta(s, grid, g, det.logodds_zref(L, g))
            lp = np.clip(lp + dp, -2.0, 3.5)
            ln = np.clip(ln + dn, -2.0, 3.5)
            # ROS form: only the returned points and the sensor origin; body returns removed by the body box
            ok = s['valid'] & ~s['is_dust']
            Pw = s['pts'][ok]
            c, sn = math.cos(pose[2]), math.sin(pose[2])
            rel = Pw - [pose[0], pose[1], ac.val('base_height')]
            Pb = np.stack([c * rel[:, 0] + sn * rel[:, 1], -sn * rel[:, 0] + c * rel[:, 1], rel[:, 2]], 1)
            keep = ~ac.body_mask(Pb)
            tc = time.time()
            scan = ac.make_scan(s['origin'], Pw[keep], ac.val('l1_sigma'), 'l1')
            hm.add(scan)
            hm.refit()
            t_core += time.time() - tc
        ref = det.clusters(lp > 0, ln > 0, grid, lp, ln, det.l_decide('l1'))
        got = hm.objects()
        seen_x = x0 + n * ds + 2.0                        # obstacles in range of the drive
        truth = [o for o in arena.obstacles if o.kind != 'column' and o.x < seen_x and abs(o.y - y0) < 1.6]

        def hits(dets, key):
            out = 0
            for o in truth:
                want = 'pos' if o.kind == 'boulder' else 'neg'
                wk = 'boulder' if o.kind == 'boulder' else 'crater'
                out += any((d[key] in (want, wk)) and math.hypot(d['x'] - o.x, d['y'] - o.y) < o.radius + 0.15
                           for d in dets)
            return out

        def false(dets, key):
            bad = 0
            for d in dets:
                if min(d['x'], d['y'], arena.L - d['x'], arena.W - d['y']) < 0.15:
                    continue
                if not any(math.hypot(d['x'] - o.x, d['y'] - o.y) < o.radius + 0.15 for o in arena.obstacles):
                    bad += 1
            return bad
        rows.append((seed, len(truth), hits(ref, 'kind'), hits(got, 'kind'), false(ref, 'kind'), false(got, 'kind'),
                     n, t_core / n))
        print(f'detector seed {seed}: {len(truth)} obstacles near the path; found by sandbox loop {rows[-1][2]}, '
              f'by HazardMap {rows[-1][3]}; false alarms {rows[-1][4]} / {rows[-1][5]}; '
              f'{t_core / n * 1000:.0f} ms per frame in HazardMap ({time.time() - t0:.0f} s total)')
    r = np.array([row[1:6] for row in rows])
    assert r[:, 2].sum() >= r[:, 1].sum() - 1 and r[:, 4].sum() <= r[:, 3].sum() + 1
    return rows


def test_filter(seeds=(3000, 3002), level='nominal'):
    drift = ac.drift
    a2 = ac.a2
    out = []
    for seed in seeds:
        t0 = time.time()
        log, marks = drift.drive(seed, level)
        rng = np.random.default_rng([seed, 99, list(a2.SLIP_LEVELS).index(level)])
        arena, frames = drift.lidar_frames(seed, log, rng)
        scale = (1.0 - a2.SLIP_LEVELS[level][0]) * (1.0 + rng.normal(0, drift.v(drift.PARAMS, 'wheel_cal_err')))
        sx, sa = drift.v(drift.PARAMS, 'tag_sigma')
        start = log[0, 1:4] + [rng.normal(0, sx), rng.normal(0, sx), np.radians(rng.normal(0, sa))]
        bias = np.radians(rng.normal(0, drift.v(drift.PARAMS, 'gyro_bias')))
        dt = a2.SIM_DT
        n_sd = np.radians(drift.v(drift.PARAMS, 'gyro_noise')) * np.sqrt(1.0 / dt)
        gyro = log[:, 6] + bias + rng.normal(0, n_sd, len(log))
        v_wheel = (log[:, 4] + log[:, 5]) / 2
        f = ac.WallFilter(scale=scale, size=(arena.L, arena.W), wall_offset=0.0, band=drift.v(drift.PARAMS, 'wall_band'))
        f.init(start, log[0, 0])
        est = np.zeros((len(log), 3))
        est[0] = start
        fi, prev = 0, 0
        T = 0.1
        for i in range(1, len(log)):
            f.predict(v_wheel[i], gyro[i], log[i, 0])
            est[i] = f.x
            while fi < len(frames) and frames[fi][0] == i:
                _, pts, ts = frames[fi]
                # the cloud arrives at the frame end; points carry their slice time in this replay
                for k in np.unique(ts):
                    m = ts == k
                    Pb = pts[m].copy()
                    Pb[:, 2] -= ac.val('base_height')
                    f.add_wall_points(Pb, log[i, 0] - T + k * T)
                if (fi + 1) % 10 == 0:
                    f.wall_fix()
                    est[i] = f.x
                fi += 1
        res = []
        for idx, kind, ok in marks:
            e = math.hypot(*(est[idx, :2] - log[idx, 1:3]))
            res.append((kind, round(e * 100, 2), round(abs(math.degrees(ac.wrap(est[idx, 2] - log[idx, 3]))), 2)))
        out.append((seed, res, dict(f.stats)))
        print(f'filter seed {seed} {level}: leg ends (kind, cm, deg) {res}; {f.stats}; {time.time() - t0:.0f} s')
    worst = max(e for _, res, _ in out for _, e, _ in res)
    assert worst < 6.0, worst
    return out


def test_planner(seeds=tuple(range(3000, 3010)), level='nominal', reveal=2.5):
    """MissionRunner on the skid-steer model with slip. Hazards are revealed when within reveal metres of base_link
    (the detector's reach is tested above), so routes planned early meet hazards late and must be replanned."""
    a2 = ac.a2
    rows = []
    for seed in seeds:
        w = a2.world(seed)
        start, goals = a2.mission(w, np.random.default_rng(seed))
        robot = a2.SkidSteer(start, level, np.random.default_rng([seed, 7]))
        model = ac.RobotModel(mult=a2.SLIP_LEVELS[level][2])
        run = ac.MissionRunner(goals[:3], model)
        known, events, contact, t = [], [], 0.0, 0.0
        orig = ac.make_world
        ac.make_world = lambda hz: _sandbox_world(hz, w)
        try:
            while run.state != 'done' and t < 240.0:
                for o in w.obstacles:
                    h = {'kind': o.kind, 'x': o.x, 'y': o.y, 'r': o.r}
                    if o.kind != 'column' and math.hypot(o.x - robot.pose[0], o.y - robot.pose[1]) < reveal \
                            and h not in known:
                        known.append(h)
                cv, cw, ev = run.step(robot.pose.copy(), t, known, 0.05)
                if ev:
                    events.append((ev[0], round(math.hypot(*(robot.pose[:2] - np.asarray(ev[1][:2]))), 3)))
                for _ in range(5):                           # 20 Hz control, 100 Hz physics
                    robot.step(cv, cw, a2.SIM_DT)
                    contact += a2.SIM_DT * (a2.clearance(w, robot.pose) < 0)
                t += 0.05
        finally:
            ac.make_world = orig
        tol = ac.v(a2.PARAMS, 'goal_tol')[0]
        ok = sum(e <= tol for _, e in events)
        replans = sum('replan' in m for _, m in run.log)
        rows.append((seed, ok, replans, round(contact, 2), round(t, 1)))
        print(f'planner seed {seed}: goals reached {ok}/3, replans {replans}, contact {contact:.2f} s, '
              f'{t:.0f} s sim time')
        if ok < 3 or contact > 0:
            print('   ' + '\n   '.join(f'{tt:.1f} {m}' for tt, m in run.log))
    assert sum(r[1] for r in rows) >= 3 * len(rows) - 1 and all(r[3] == 0 for r in rows)
    return rows


def _sandbox_world(hazards, w):
    """World in the sandbox arena's own geometry (6.8 m long, its column) for this offline check."""
    obs = [ac.a2.Disc(h['kind'], h['x'], h['y'], h['r']) for h in hazards]
    obs += [o for o in w.obstacles if o.kind == 'column']
    return ac.a2.World(seed=ac.world_key(obs), L=w.L, W=w.W, obstacles=obs, berm=w.berm)


def test_closed(seeds=(3000, 3001, 3002), level='nominal', legs=3):
    from arena import Arena
    from sensors import Lidar
    import selfmask
    import localize as lz
    drift, a2 = ac.drift, ac.a2
    xyz, R, table = lz.parts('front45')
    rows = []
    for seed in seeds:
        t_wall = time.time()
        w = a2.world(seed)
        arena = Arena(np.random.default_rng(seed))
        start, goals = a2.mission(w, np.random.default_rng(seed))
        rng = np.random.default_rng([seed, 5])
        robot = a2.SkidSteer(start, level, np.random.default_rng([seed, 7]))
        lid = Lidar('l1', mount_xyz=xyz, mount_R=R, body=lz._CACHE['body'])
        scale = (1.0 - a2.SLIP_LEVELS[level][0]) * (1.0 + rng.normal(0, drift.v(drift.PARAMS, 'wheel_cal_err')))
        sx, sa = drift.v(drift.PARAMS, 'tag_sigma')
        bias = np.radians(rng.normal(0, drift.v(drift.PARAMS, 'gyro_bias')))
        n_sd = np.radians(drift.v(drift.PARAMS, 'gyro_noise')) * np.sqrt(1.0 / a2.SIM_DT)
        f = ac.WallFilter(scale=scale, size=(arena.L, arena.W), wall_offset=0.0, band=drift.v(drift.PARAMS, 'wall_band'))
        f.init(np.asarray(start) + [rng.normal(0, sx), rng.normal(0, sx), np.radians(rng.normal(0, sa))], 0.0)
        hm = ac.HazardMap(size=(arena.L, arena.W))
        model = ac.RobotModel(mult=a2.SLIP_LEVELS[level][2])
        run = ac.MissionRunner(goals[:legs], model)
        hazards, events, contact, err = [], [], 0.0, []
        orig = ac.make_world
        ac.make_world = lambda hz: _sandbox_world(hz, w)
        t, k = 0.0, 0
        cv = cw = 0.0
        try:
            while run.state != 'done' and t < 300.0:
                if k % 5 == 0:                                           # 20 Hz control on the estimate
                    n_log = len(run.log)
                    cv, cw, ev = run.step(f.x.copy(), t, hazards, 0.05)
                    if os.environ.get('CLOSED_DEBUG') and any('no route' in m for _, m in run.log[n_log:]):
                        print(f'   t {t:.1f} est {np.round(f.x, 3)} true {np.round(robot.pose, 3)} hazards '
                              f'{[(h["kind"][0], round(h["x"], 2), round(h["y"], 2), round(h["r"], 2)) for h in hazards]}')
                    if ev:
                        events.append((ev[0], math.hypot(*(robot.pose[:2] - np.asarray(ev[1][:2])))))
                vb, wb = robot.step(cv, cw, a2.SIM_DT)
                t += a2.SIM_DT
                k += 1
                contact += a2.SIM_DT * (a2.clearance(w, robot.pose) < 0)
                f.predict((robot.vl + robot.vr) / 2, wb + bias + rng.normal(0, n_sd), t)
                if k % 10 == 0:                                          # 10 Hz L1 frame at the true pose
                    pose = (robot.pose[0], robot.pose[1], np.degrees(robot.pose[2]))
                    s = lid.scan(arena, pose, rng)
                    ok = s['valid'] & ~s['is_dust']
                    Pw = s['pts'][ok]
                    c, sn = math.cos(robot.pose[2]), math.sin(robot.pose[2])
                    rel = Pw - [robot.pose[0], robot.pose[1], ac.val('base_height')]
                    Pb = np.stack([c * rel[:, 0] + sn * rel[:, 1], -sn * rel[:, 0] + c * rel[:, 1], rel[:, 2]], 1)
                    Pb = Pb[~ac.body_mask(Pb)]
                    f.add_wall_points(Pb, t)
                    T_ab = ac.base_T(*f.x)                              # placed with the ESTIMATED pose
                    origin_b = np.array([c * (s['origin'][0] - robot.pose[0]) + sn * (s['origin'][1] - robot.pose[1]),
                                         -sn * (s['origin'][0] - robot.pose[0]) + c * (s['origin'][1] - robot.pose[1]),
                                         s['origin'][2] - ac.val('base_height')])
                    hm.add(ac.make_scan(ac.apply_T(T_ab, origin_b[None])[0], ac.apply_T(T_ab, Pb), ac.val('l1_sigma'),
                                        'l1'))
                if k % 100 == 0:                                         # 1 Hz: wall fix, ground refit, hazards
                    f.wall_fix()
                    hm.refit()
                    hazards = hm.objects()
                    err.append(math.hypot(*(f.x[:2] - robot.pose[:2])))
        finally:
            ac.make_world = orig
        tol = ac.v(a2.PARAMS, 'goal_tol')[0]
        ok = sum(e <= tol for _, e in events)
        truth = [o for o in w.obstacles if o.kind != 'column']
        found = sum(any(math.hypot(h['x'] - o.x, h['y'] - o.y) < o.r + 0.15 for h in hazards) for o in truth)
        false = sum(not any(math.hypot(h['x'] - o.x, h['y'] - o.y) < o.r + 0.15 for o in w.obstacles) for h in hazards)
        rows.append((seed, ok, len(events), round(contact, 2), float(np.median(err)), float(np.max(err)), found, false))
        print(f'closed seed {seed}: goals reached {ok}/{legs} (arrival errors {[round(e * 100, 1) for _, e in events]} cm), '
              f'contact {contact:.2f} s, pose error median {np.median(err) * 100:.1f} cm max {np.max(err) * 100:.1f} cm; '
              f'hazards {len(hazards)} ({found} of {len(truth)} arena obstacles, {false} with none near); '
              f'{f.stats}; {t:.0f} s sim, {time.time() - t_wall:.0f} s wall')
        if ok < legs or contact > 0:
            print('   ' + '\n   '.join(f'{tt:.1f} {m}' for tt, m in run.log))
    return rows


if __name__ == '__main__':
    which = sys.argv[1:] or ['tags', 'detector', 'filter', 'planner']
    for name in which:
        globals()['test_' + name]()
