# Start-zone tag test

Question (plan Step 2): how well does the rear camera find the robot's start pose from the team's own tags, and do
two perpendicular tags beat one tag or two tags on one wall? No source we opened measured this
(`research/step1/11-fiducial-gaps.md`). Labels: Verified, Inferred, Assumed; every number the test uses is in
`tag_start.PARAMS` with its label and source.

## The test (`tag_start.py`)

- Rules: tags go on the bin frame along two sides of the starting zone (2027 guidebook 13.7, Verified); the zone is in
  an arena corner (11.2, Verified), so the two sides are taken as the walls x = 0 and y = 0 (Inferred). Start position
  and heading are random (13.9.3, Verified), and the robot may move inside the zone to localize (16.5, Verified).
- Robot and camera: rear camera at the team URDF pose (0.42 m behind base_link, 0.37 m up, facing back, 15 deg up),
  1280 x 720, 1.2 rad horizontal field of view (Verified, `sim/robot/uic_bot_2027.xacro`).
- Zone 2.0 x 2.0 m (Assumed, Figure 9 unreadable); the robot's centre stays 0.66 m inside it, so the camera is
  0.25-0.9 m from a wall. Tags 16 cm (Assumed), corner noise 0.5 px, decoded if at least 20 px wide and seen within
  70 deg of face-on, tag position known to 5 mm and 1 deg after setup (all Assumed).
- Layouts: `one` (one tag on wall y = 0), `two_coplanar` (two on that wall), `two_perpendicular` (one per wall),
  `four` (two per wall). Pose: OpenCV IPPE for tags in one plane (lowest reprojection error of its two answers, as a
  robot would pick), SQPnP + Levenberg-Marquardt when both walls are in view.
- Per setup: 1500 random start poses, the same poses and noise for every layout (paired). Reported: share of poses
  that see a tag at the drawn heading, share that see none at any heading, the in-place turn needed, the robot's
  position and heading error, bootstrap 95% intervals.

`./sweep.sh results/sweep_v1 1500` runs 18 camera and tag setups (field of view 1.2 or 2.0 rad, pitch 15, 0 or -15
deg, tag centre 0.3, 0.6 or 0.9 m high); `python3 summarize.py results/sweep_v1` writes `results/sweep_v1/table.md`.

## Results (2026-10-12, sim, Inferred)

1. **The current rear camera cannot see tags from inside the zone.** With the URDF camera (1.2 rad, 15 deg up), tags
   0.3 or 0.6 m high are never in view from any heading; at 0.9 m, 10-33% of start positions still see none after a
   full turn. Close to the wall the tags fall below the image.
2. **A wide lens pitched down fixes coverage.** At 2.0 rad (about 115 deg, pinhole, distortion ignored), 15 deg down,
   with tags 0.3 m high, every start pose reaches a tag by turning in place with two or more tags (4% never do with
   one tag). Level, the same lens leaves 10-33% of poses with no tag in view except with four tags.
3. **Two perpendicular tags do not improve accuracy; they reduce turning.** The camera almost never sees both walls
   at once (0-1% of views with two tags, 12-15% with four), so a pose comes from one wall's tags. Paired on the same
   poses (2.0 rad, 15 deg down, tags 0.3 m): perpendicular minus one tag, median position error +0.1 cm
   [95% CI -0.1, +0.3]; perpendicular minus coplanar pair +0.2 cm [+0.1, +0.5]. The gain is coverage: median turn
   60 deg with one tag, 35 with a coplanar pair, 20 with a perpendicular pair, 0 with four.
4. **Accuracy is about 1.3-2.5 cm median and 4-6 cm at the 95th percentile, heading under 1.5 deg median**, and it
   is set mostly by how well the tags' own positions are known (the 5 mm / 1 deg Assumed), not by the tag count.
5. **Avoid tags at camera height.** With tags 0.6 m high (camera 0.64 m) and the wide lens, single-plane poses pick
   the wrong one of the two IPPE answers 4-9% of the time (heading off by more than 5 deg); at 0.3 m, under 1%.

Best setup found: **four tags (two per wall) about 0.3 m high, a wide-angle rear camera (about 115 deg) level or
pitched down**: seen at the drawn heading 55% of the time, no turn needed at the median, position error 1.3 cm
median [1.2, 1.4] and 4.1 cm at the 95th percentile, no heading errors over 5 deg in 1500 starts.

Caveats: the zone size and tag placement accuracy are Assumed and drive the numbers; lens distortion, motion blur,
lighting on BP-1 and partial occlusion by the robot are not modelled; the table is in `results/sweep_v1/table.md`.
