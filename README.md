# BagPath

BagPath is a rentable attachment that makes any suitcase follow its owner hands-free through busy airports. A HuskyLens camera tracks the traveler and an Arduino drives two DC motors to hold a fixed following distance, so travelers keep the suitcase they already own instead of buying an $800+ smart suitcase.

Built by a 4-person team (Ari Bellman, Josh Greenfield, Nadav Levine, Daniel Lerner) for CIJE Innovation Day 2026, where it won **1st Place, Best Use of AI/ML**. Nadav led data analytics and computer vision: the on-device tracking code and a 28-traveler survey (89% found it useful) that validated a $5–$15 per-trip rental price.

**Portfolio and demo:** [bagpath.lovable.app](https://bagpath.lovable.app)

**Status:** the core follow-me loop (detect leader, estimate distance, steer motors to close/hold distance) works and has been calibrated against the team's own hardware. The planned UWB radio upgrade (see below) has **not** been built — current tracking is camera-only and requires line of sight.

## Hardware

- **Microcontroller:** Arduino Uno
- **Vision sensor:** HuskyLens (I2C)
- **Motor driver:** L298N, two DC motors

### Pin mapping (`BagPath_Main.ino`)

| Signal | Pin | Purpose |
|---|---|---|
| ENA | 5 | Right motor speed (PWM) |
| IN1 | 6 | Right motor direction |
| IN2 | 7 | Right motor direction |
| IN3 | 8 | Left motor direction |
| IN4 | 9 | Left motor direction |
| ENB | 10 | Left motor speed (PWM) |

Right motor = L298N OUT1/OUT2, left motor = OUT3/OUT4. If a side spins backward, flip `REVERSE_LEFT_MOTOR` / `REVERSE_RIGHT_MOTOR` rather than rewiring.

### HuskyLens setup

Connected over I2C (`Wire.begin()`), running in **Object Tracking** mode (`ALGORITHM_OBJECT_TRACKING`). Rather than HuskyLens's dedicated face-recognition algorithm, BagPath uses Object Tracking's "learn" feature and simply trains it on whatever the current leader marker is (see Tracking approach below) — the same underlying algorithm handles all three generations of marker.

### Calibration constants

Distance is estimated from the tracked object's apparent size, calibrated once at a known distance:

```
REAL_W_CM = 7.0        // leader marker's real-world width (cm)
REAL_H_CM = 13.5       // leader marker's real-world height (cm)
CAL_DIST_CM = 30.0     // calibration was measured at 30cm from the lens
CAL_W_PX = 173.0       // marker's pixel width at that distance
CAL_H_PX = 173.0       // marker's pixel height at that distance
```

These feed a simple pinhole-camera focal-length estimate (`fX`, `fY`) used to convert apparent size back into distance at runtime. **Recalibrate these four values** any time the tracked marker or mounting geometry changes — they're specific to the object HuskyLens has been trained on.

Distance thresholds: robot stops within `STOP_DIST_CM = 12cm`, drives forward once the leader is beyond `GO_DIST_CM = 18cm`.

## Tracking approach — and why it changed

The leader-tracking target evolved through three stages as the team ran into real-world limitations:

1. **Cardboard marker.** The earliest working version (`Code Files` history) tracked a small printed cardboard square (labeled "B") held by the leader. Simple and reliable in testing, but required the leader to visibly hold a physical object at all times.
2. **AprilTag.** The team moved to an AprilTag-style fiducial marker for more robust, uniquely-identifiable tracking — still required a printed tag on the leader, but was more resistant to false-positive detections than plain cardboard.
3. **Face recognition (current).** The current build trains HuskyLens's Object Tracking algorithm on the leader's face instead of a printed marker, so the leader doesn't need to carry or wear anything. This is the mode `BagPath_Main.ino` is calibrated for today.

Each stage kept the same underlying control loop (`readBestTarget` → `estimateDistanceCm` → `computeForwardSpeed`/turn bias → ramped motor output) — only what HuskyLens was trained to recognize changed.

## Planned: UWB radio upgrade

Camera-based tracking requires unbroken line of sight between HuskyLens and the leader — crowds of other travelers, pillars, or the leader briefly turning a corner can break tracking. The team's next planned step is adding Ultra-Wideband (UWB) radio ranging between the robot and a leader-worn tag, which would let BagPath maintain distance and bearing to the leader without needing to see them. **This has not been implemented yet** — the current repo is camera-only.

## Repo contents

- `BagPath_Main/BagPath_Main.ino` — the current, working follow-me sketch (motor control, HuskyLens tracking, distance-based speed/steering).
- `HuskyLens_Serial_Test/`, `HuskyLens_XY_Height_Test/` — standalone test sketches used to verify HuskyLens serial communication and read raw X/Y/height data during calibration.
- `docs/BagPath_Block_Diagram.pdf` — system block diagram.
- `images/` — prototype photos.

Additional design files, meeting notes, pitch materials, and raw demo photos/video live outside this repo. Demo footage and the team's portfolio page are hosted at [bagpath.lovable.app](https://bagpath.lovable.app).

## Running it

1. Wire the Uno, HuskyLens (I2C), and L298N per the pin mapping above.
2. Open `BagPath_Main/BagPath_Main.ino` in the Arduino IDE (or via `arduino-cli`).
3. In the HuskyLens, switch to Object Tracking mode and "learn" the leader (face or marker) as the tracked object.
4. Re-measure and update the four calibration constants for your specific setup and mounting distance.
5. Upload and power the robot — it should hold position behind the leader once tracking locks on.
