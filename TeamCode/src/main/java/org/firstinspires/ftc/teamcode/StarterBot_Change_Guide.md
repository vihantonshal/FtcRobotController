# Changing the StarterBot programs (BIOBUZZ 2026–27)

Our robot runs the goBILDA starter-kit programs, changed for our GripForce 104 mm mecanum wheels.
Each part of the robot is its own class in the `subsystems` folder, with its own tuning numbers,
so each team can own one file:

| File | Driver Station name | What it does | Owner |
|---|---|---|---|
| `BioBuzzStarterBotAuto.java` | BioBuzz StarterBot Auto | AUTO: starts in front of the HIVE (or, with `USE_LOOK = true`, drives to where the camera can see the up CELL first), turns toward it (searching if needed), launches for 10 s, then parks in the LOADING ZONE | Auto team |
| `BioBuzzStarterbotTeleopMecanum.java` | Mec BioBuzz StarterBot Teleop | TeleOp: reads the gamepad and tells each part what to do | Drive team |
| `subsystems/MecanumDriveSubsystem.java` | (not an OpMode) | Four drive motors: mecanum formula, drive/strafe/rotate moves, drive PIDF | Drive team |
| `subsystems/IntakeSubsystem.java` | (not an OpMode) | Intake roller and the two corner servos, intake PIDF | Intake team |
| `subsystems/LauncherSubsystem.java` | (not an OpMode) | Launcher and windmill, launcher PIDF and speeds | Launcher team |
| `subsystems/AprilTagVision.java` | (not an OpMode) | Webcam, HIVE tags 30–45, aiming numbers | Vision team |
| `subsystems/RobotSwitches.java` | (not an OpMode) | On/off switch for each part | Everyone (ask first) |
| `BioBuzzMotorMaxSpeed.java` | BioBuzz Motor Max Speed | Tuning tool: measures a motor's top speed and suggests PIDF numbers | Everyone |

To find a setting, search for its name (Ctrl+F in Android Studio). Line numbers change every time
someone edits the file, so this guide doesn't use them.

## Rules before you change anything

1. **Change one thing at a time.** Then test. If two things change and the robot misbehaves, you won't know which one did it.
2. **Each number lives in one place: its subsystem.** The TeleOp and the Auto both use the same subsystem classes, so a change there changes both programs.
3. **Only change your own team's file.** If you need something from another part (a new method, a new number), ask its owner. Then two people never edit the same file at the same time.
4. **Test with the wheels off the ground first** (robot on a block), with one hand on STOP.
5. **Write it down.** Note in the engineering notebook what you changed, from what to what, and what happened. Commit the code with a message like `Launcher: TARGET_VELOCITY 1250 -> 1300`.

## Controls (TeleOp)

The **driver** uses gamepad 1. The **operator** uses gamepad 2. With `TWO_DRIVERS = false` (top of
the TeleOp), one person does everything on gamepad 1; the buttons don't overlap.

| Who | Control | Action |
|---|---|---|
| Driver | Left stick | Drive forward/back and strafe left/right |
| Driver | Right stick left/right | Turn |
| Driver | Left bumper (hold) | Turn toward the closest AprilTag (you can still drive and strafe) |
| Driver | X | Slow mode on/off (40% speed, for lining up) |
| Driver | Y | Field-centric on/off. On: stick up = away from the driver, whichever way the robot faces |
| Driver | Back / Share | Reset "forward": point the robot away from you, then press |
| Operator | D-pad up (hold) | Auto launch: spin up, drive to the nearest saved good launch pose for the up CELL in view, then launch until released. While held the robot drives itself (driver sticks ignored). `LaunchAssist.AUTO_LAUNCH = false` makes it drive only |
| Operator | Right trigger / left trigger | Intake in / out (harder pull = faster) |
| Operator | Right bumper | Launcher on/off. It keeps spinning; the gamepad **rumbles** when it is fast enough |
| Operator | A (hold) | Feed balls (only the operator launches). Feeding starts once the launcher is within `FEED_MARGIN` (50 ticks/s, goBILDA's own value) of its target speed, then keeps running without a pause until A is released |
| Operator | B | "That launch scored": save the last launch's pose as a good launch pose |

### All buttons: Logitech and REV gamepads

Every button on both gamepads, with `TWO_DRIVERS = true`. "—" means the button isn't used.

| Logitech | REV | Driver (gamepad 1) | Operator (gamepad 2) |
|---|---|---|---|
| Left stick | 🕹️ L stick | Drive and strafe | — |
| Right stick | 🕹️ R stick | ◀ ▶ Turn | — |
| A | ✕ Cross | — | Hold: feed balls, roller runs nonstop once at speed |
| B | ○ Circle | Before START: we are RED | Save the last launch as a good pose |
| X | ▢ Square | Before START: we are BLUE. After START: slow mode on/off | — |
| Y | △ Triangle | Does nothing now | — |
| LB | L1 | Hold: turn toward the up CELL | — |
| RB | R1 | — | Launcher on/off |
| LT | L2 | — | Hold: intake out |
| RT | R2 | — | Hold: intake in |
| D-pad up | ▲ | — | Hold: auto launch |
| D-pad down | ▼ | — | — |
| D-pad left | ◀ | — | Flywheel a bit slower |
| D-pad right | ▶ | — | Flywheel a bit faster |
| Back | Share | Does nothing useful now | — |
| Start | Options | Start + A / Options + ✕: connect as gamepad 1 | Start + B / Options + ○: connect as gamepad 2 |

## Launch log and good launch poses (practice)

`subsystems/LaunchAssist.java` remembers where the robot was when a launch scored.

1. **A good launch pose is what the camera sees**: which HIVE CELL, and how far and at what angle its
   opening is. It doesn't matter where TeleOp started or how many trips the robot made.
2. Drive, spin up, feed with A. **Every launch** (each press of A with the launcher on) is written to
   the log with the pose, the HIVE CELL the camera sees (cluster, distance, bearing, yaw) and the flywheel
   speed and direction. If the camera missed the CELL in that one frame, the last pose that saw it
   (at most 0.5 s old) is used.
3. When a ball scores, the operator presses **B**: that launch is saved as a **good launch pose**.
   A launch with no CELL in view is not saved ("Not saved: the camera saw no CELL"), and pressing B
   twice saves it only once. Saved poses load again at every INIT, so they add up over practice sessions.
4. The Driver Hub always shows **Launch pose: OK** or how far off the robot is ("8.3 in off, turn 5° left"),
   and **Flywheel: FORWARD 1236 ticks/s (2649 RPM)**. A "flywheel spins BACKWARD" warning appears if the
   launcher runs the wrong way. The driver's gamepad LED is green when the pose is OK, red when not,
   blue when there is no target: no up CELL in view, or no saved pose for it yet (PS4/PS5 gamepads).
5. The driver **holds the right bumper** to let the robot drive itself to the nearest good pose; the
   gamepad buzzes when it gets there. It matches the distance and angle to the same HIVE CELL as the
   saved pose. It only works while the camera sees that CELL: if the LED is blue, turn toward the HIVE first (two short blips on the gamepad = the camera found an up CELL; purple = it sees the CELL but no good pose is saved for it yet).

**Reviewing the log:** the file is `/sdcard/FIRST/BioBuzz/launch_log.csv` on the Control Hub. Copy it to a
laptop with a USB-C cable (Windows File Explorer: `This PC\Control Hub v1.0\Internal shared storage\FIRST\BioBuzz\`)
or `adb pull /sdcard/FIRST/BioBuzz/launch_log.csv`, and open it in a spreadsheet. REV Hardware Client can't
do this (it has no file browser); step by step in `FtcRobotController/docs/getting-log-files.md`.
Each row is a LAUNCH or a SUCCESS with the time, odometry pose (for reference only), tag data,
launcher ticks/s, RPM and direction.
Delete the file to start fresh (for example at a new field).

**Tuning (FTC Dashboard, group `LaunchAssist`):** `TAG_RANGE_TOL_IN`, `TAG_BEARING_TOL_DEG`,
`TAG_YAW_TOL_DEG` decide what counts as "OK";
`ALIGN_KP_*` and `ALIGN_MAX_POWER` decide how hard auto-align drives. **First test:** hold the right
bumper near a tag with a saved pose. If the robot slides the wrong way to fix the tag angle, set
`YAW_STRAFE_SIGN = -1`.

Field-centric needs the Control Hub's built-in IMU (`imu` in the configuration). Without it the
TeleOp still drives, robot-centric. If field-centric turns the wrong way, fix `HUB_LOGO` and `HUB_USB`
in `MecanumDriveSubsystem` to match how the Control Hub is mounted.

Driver-feel numbers in `MecanumDriveSubsystem` (also live in FTC Dashboard under
`MecanumDriveSubsystem`): `DEADBAND` 0.05 (stick values below this count as 0),
`SLOW_MODE_SCALE` 0.4, `MAX_ACCEL_PER_SEC` 4 (0 to full speed in 0.25 s, so the robot doesn't jerk),
`FIELD_CENTRIC_AT_START`. At the top of the TeleOp (FTC Dashboard: `BioBuzzStarterbotTeleopMecanum`):
`TWO_DRIVERS`, `READY_RUMBLE_BLIPS` (3 short buzzes on the operator's gamepad when the flywheel reaches its speed).

The TeleOp's `loop()` only says WHAT each part should do, in four steps: look (camera, odometry,
launch pose), driver, operator, telemetry. HOW each part does it is in its subsystem:
`drive.driveWithSticks()` (deadband, slow mode, smooth speed-up), `launcher.toggleOn()` / `run()` /
`justBecameReady()`, `launchAssist.update()` / `recordLaunch()` / `saveLastLaunch()` / `showOnLed()`,
`intake.runTeleop()`.

## Robot configuration names

These names must match the Driver Hub configuration exactly, or the program stops with an error
when you press INIT:

`front_left_motor`, `front_right_motor`, `back_left_motor`, `back_right_motor`, `intake`,
`launcher`, `windmillServo`, `left_intake_servo`, `right_intake_servo`, and the webcam `Webcam 1`.
Optional: `feedWheel`, the continuous rotation servo of the feed wheel beside the Runway lead-in. It runs
whenever the windmill feeds; when it is missing from the configuration, everything works as before.
The Control Hub's built-in IMU is called `imu` (the Driver Hub adds it when you scan).

Plug each motor's encoder cable into the **Encoder** port with the same number as its **Motor**
port, on the same hub.

## Our motors

From the goBILDA StarterBot for BIOBUZZ assembly instructions (3200-2627-0003) plus our mecanum drive:

| Part | Motor (SKU) | Output speed | Encoder ticks per output turn | Top speed | Spins |
|---|---|---|---|---|---|
| Drive ×4 | 5203-2402-0019, 19.2:1 | 312 RPM | 537.7 | ≈ 2796 ticks/s | GripForce 104 mm mecanum wheels |
| Intake | 5203-2402-0051, 50.9:1 | 117 RPM | 1425.1 | ≈ 2779 ticks/s | 48 mm Gecko roller through a 100-tooth → 20-tooth gear (≈ 585 RPM) |
| Launcher | 5203-2402-0019 converted to 1:1 (conversion coupler) | ≈ 6000 RPM | 28 | ≈ 2800 ticks/s | 96 mm Hogback wheel + goBILDA 3628-0032-0082 steel flywheel (Ø82, 152 g, 8 × M4 × 12 thread-forming screws into the wheel's side; ~2.5% speed drop per shot instead of ~6–7%); 1250 ticks/s ≈ 2679 RPM |
| Servos | 2000-0025-0003 Speed ×2 (+1 for the feed wheel), 2000-0025-0002 Torque ×1 | — | — | — | 72 mm Gecko intake wheels, Ø36 printed TPU feed wheel, windmill feeder |

The kit has three 19.2:1 motors, so the four-motor mecanum drive needs two more, and six motors need
an Expansion Hub (the Control Hub has four motor ports). The intake gear order is read from the
manual's step 33 drawing; check it on the robot.

## Feature switches: drive while a part is off the robot

`subsystems/RobotSwitches.java` has an on/off switch for each part. Change one to `false`, rebuild and
install. The TeleOp, the Auto and the Motor Max Speed tool all read them.

| Switch | Turns on/off | When it is `false` |
|---|---|---|
| `USE_DRIVE` | the four mecanum drive motors | `MecanumDriveSubsystem` doesn't look for them and the robot never moves, so you can test only the camera / AprilTags. The Auto's drive and turn steps count as done at once. |
| `USE_INTAKE` | intake motor, left and right intake servos | `IntakeSubsystem` doesn't look for them, so they can be unplugged or missing from the configuration. |
| `USE_LAUNCHER` | launcher motor, windmill servo | The right bumper does nothing; the Auto skips LAUNCH. |
| `USE_CAMERA` | webcam, AprilTag aiming | The left bumper does nothing; the Auto skips AIM. |
| `USE_DASHBOARD` | FTC Dashboard graphs, camera stream, TeleOp telemetry | The TeleOp sends **no telemetry at all**; the Auto still shows its state on the Driver Station. **Set to `false` for competition.** |
| `USE_LAUNCH_ASSIST` | TeleOp launch assist: launch log, saving good poses (operator B), auto-align (driver right bumper) | Nothing is logged or saved, the align button does nothing and the driver LED is left alone. The Auto doesn't use it. |

While the Dashboard is on, the INIT telemetry line "Switched on" shows which parts are on, so
nobody wonders why the intake isn't moving. **Turn every switch back to `true` before competition**, except `USE_DASHBOARD`.

## Safe knobs, by file

### `BioBuzzStarterBotAuto.java` (the plan)

| You want to… | Change this | Now | Notes |
|---|---|---|---|
| Launch for a shorter or longer time | `LAUNCH_SECONDS` | 10 s | Counted from the start of the LAUNCH step. |
| Drive to a LOOK spot first | `USE_LOOK` | false | false = the robot is placed in front of the HIVE and starts at AIM from where it stands (no LOOK, no other-side LOOK spot). true = start against the ALLIANCE wall and drive to the LOOK spot first. |
| Turn to the CELL before launching | `AIM_TURN` | false | false = the robot is placed already pointing at the HIVE: AIM doesn't turn and SEARCH never runs, it goes straight to LAUNCH (feeds once the flywheel is fast enough). true = AIM turns the launcher to the up CELL and searches ±45° if it can't see it. |
| Look from somewhere else | `LOOK_FORWARD_IN`, `LOOK_RIGHT_IN` | 18 in, 30 in | Where the robot goes before aiming, measured from where it stood at INIT: first forward (off the wall, clear of the FLOWER), then right. The up CELL's AprilTags are on its bottom face, so they face down and out toward the end of the field beyond it, and from the middle of the wall the camera sees them from behind. The same move for red and blue. Both 0 = aim from the start spot. |
| Faster or slower flywheel in AUTO only | `AUTO_SPEED_ADD` (FTC Dashboard: BioBuzzStarterBotAuto, set before INIT) | +100 ticks/s | Added to `TARGET_VELOCITY` (1179) in the Auto only, so the Auto shoots at 1279 (scored well from the start spot in front of the HIVE, touching our wall); the TeleOp keeps 1179. 0 = the same speed as the TeleOp. |
| Give aiming more or less time | `FIND_TIMEOUT_SECONDS` | 6 s | Counted from the end of LOOK, for AIM and SEARCH together. When it runs out, the robot turns back to its start heading and launches anyway. |
| Search wider or narrower | `SEARCH_ANGLE_DEG` | 45° | If the up CELL isn't in view for 0.5 s, SEARCH turns in place this far left, then right, until it sees it. |
| Search faster or slower | `SEARCH_TURN_POWER` | 0.2 | Slower gives the camera time to see the CELL while turning. |
| Park somewhere else | `START_FROM_WALL_IN`, `PARK_FROM_WALL_IN`, `PARK_SIDEWAYS_IN` | 0 in (touching the wall), 7.5 in, 34 in | The robot starts `START_FROM_WALL_IN` out from our ALLIANCE wall (back edge to wall), facing the HIVE, and drives ONE diagonal line to `PARK_FROM_WALL_IN` from the wall and `PARK_SIDEWAYS_IN` sideways (forward from the wall; backward only if it starts farther out than it parks). Sideways is to our alliance's LOADING ZONE: LEFT for both alliances (Manual Figure 9-2: red's zone is at the far end of the red wall, blue's at the audience end of the blue wall). `PARK_DIAGONAL_FORWARD_IN` (9 in) is how far from the wall the robot's back edge ends, in one straight diagonal while going 34 in sideways (about 15°). PARK scores only if the robot is at least partially in the LOADING ZONE (Manual 10.5.4, Figure 10-7): the zone is about 23 in wide by 11 in deep, tape included, so 9 puts the back edge on the tape line with room for the 1.5 in stopping tolerance. Keep it at 10 or less. Pick the alliance at INIT (gamepad1 B = RED, X = BLUE); not picked = the robot doesn't park. Measure on a real field. |
| Park faster or slower | `GOTO_MAX_POWER` (`MecanumDriveSubsystem`, also LOOK) | 0.4 | 0 to 1. PARK drives from wherever odometry says the robot is, so where AIM and SEARCH left it doesn't matter. |
| Give parking more or less time | `PARK_TIMEOUT_SECONDS` | 6 s | After this the robot stops wherever it is. |

### `subsystems/MecanumDriveSubsystem.java` (drive team)

| You want to… | Change this | Now | Notes |
|---|---|---|---|
| Make turns the right size | `TRACK_WIDTH_MM` | 402 | Ask for 90° with `rotateAngle()`. Turns too little: make it bigger. Turns too much: make it smaller. |
| Make strafes the right length | `STRAFE_MULTIPLIER` | 1.0 | Ask `strafeDistance()` for 500 mm and measure. New value = 500 ÷ measured distance. |
| Drive slower overall | `MAX_TICKS_PER_SEC` | 2500 | The speed a full stick asks for. |

If one wheel spins the wrong way when you push **forward**, fix its `setDirection(...)` line in the
constructor, not the formula in `drive()`. Because both programs use this class, one fix is enough.

The distance math: one wheel turn moves the robot π × 104 mm ≈ 327 mm, and the motor counts 537.7
ticks per turn, so **`ticksPerMm()` = 537.7 ÷ 327 ≈ 1.65 ticks per mm**. The class works this out
from `WHEEL_DIAMETER_MM` and `TICKS_PER_REV`.

### `subsystems/AprilTagVision.java` (vision team)

| You want to… | Change this | Now | Notes |
|---|---|---|---|
| Aim faster or gentler | `AIM_TURN_GAIN` | 0.01 | Turn power = gain × angle. Too big and the robot wobbles past the tag. |
| Limit the aim turn speed | `AIM_MAX_TURN` | 0.3 | 0 to 1. |
| Stop aiming sooner | `AIM_TOLERANCE_DEG` | 2° | Close enough counts as lined up. The Auto's AIM step uses it too. |

### `subsystems/LauncherSubsystem.java` (launcher team)

| You want to… | Change this | Now | Notes |
|---|---|---|---|
| Shoot faster or slower | `TARGET_VELOCITY` | 1381 ticks/s | ≈ 2959 RPM (= `NEAR_VELOCITY`). Used until the camera sees a CELL; after that the speed follows the distance (below). |
| Feed only when fast enough | `FEED_MARGIN` | 50 ticks/s | The roller waits until the launcher is within this of its target speed (1129 for 1179). 50 is goBILDA's own StarterBot rule; with PIDF (40, 0, 0, 12.5) the flywheel settles 30-70 short, and 25 kept the roller waiting. |
| Speed for the distance | `NEAR_RANGE_IN`, `NEAR_VELOCITY`, `FAR_RANGE_IN`, `FAR_VELOCITY` | 67 in → 1186, 83 in → 1223 (fitted to the 11 good launches in `launch_poses_default.csv`, Runway v10) | See "Distance-based flywheel speed". The mecanum TeleOp uses it when `RobotSwitches.USE_RANGE_SPEED` is true (default false); the Auto only with `USE_DISTANCE_SPEED = true`. The Driver Hub's "Shot speed" line says BY RANGE or FIXED. |
| Feed slower | `WINDMILL_POWER` | 1.0 | 0 to 1. Lower it if balls jam. |
| Feed wheel power and direction | `FEED_WHEEL_POWER` | 1.0 | Seen from above, the feed wheel must turn clockwise (pushing the ball toward the launcher). Use -1.0 if it turns the wrong way. |

### `BioBuzzStarterbotTeleopMecanum.java` (buttons)

| You want to… | Change this | Now | Notes |
|---|---|---|---|
| One person drives everything | `TWO_DRIVERS` | true | false = operator controls move to gamepad 1. |
| Use a different launcher button | `operator.right_bumper` | right bumper | Launcher on/off toggle. |
| Use a different feed button | `operator.a` | A | Hold to feed. Also change `operator.aWasPressed()`, which logs the launch. |
| Use a different aim button | `driver.left_bumper` | left bumper | In `loop()` and the "Aim" telemetry line. |
| Swap which trigger pulls in | `operator.right_trigger - operator.left_trigger` | right = in | Swap the two names. |
| Make strafing or turning go the other way | `driver.left_stick_x` or `driver.right_stick_x` in `MecanumDriveSubsystem.driveWithSticks()` | stick right = right | Put a minus in front. |
| Slower slow mode | `SLOW_MODE_SCALE` (`MecanumDriveSubsystem`) | 0.4 | 0 to 1. |
| Snappier or smoother driving | `MAX_ACCEL_PER_SEC` (`MecanumDriveSubsystem`) | 4 | Bigger = snappier, smaller = smoother. |
| Robot creeps with sticks let go | `DEADBAND` (`MecanumDriveSubsystem`) | 0.05 | Make it a little bigger. |

## PIDF: velocity control for the drive, intake and launcher

Every motor runs in `RUN_USING_ENCODER` mode. We ask for a **speed** in ticks per second, and the
motor controller uses the PIDF numbers to push harder or softer until the motor holds that speed.

Each subsystem keeps its PIDF numbers in one line at the top of its file:

| Motor | Where the numbers are | Now |
|---|---|---|
| Four drive motors | `PIDF` and `MAX_TICKS_PER_SEC` in `MecanumDriveSubsystem` | P 1.17, I 0.117, D 0, F 11.7, max 2500 |
| Intake | `PIDF` and `MAX_TICKS_PER_SEC` in `IntakeSubsystem` | same starting values |
| Launcher | `PIDF` in `LauncherSubsystem` | P 40, I 0, D 0, F 12.5 (tuned values from goBILDA) |

```java
public static PIDFCoefficients PIDF = new PIDFCoefficients(40, 0, 0, 12.5);   // p, i, d, f
// sent at INIT, and again whenever a number changes; the Control Hub runs the PIDF loop itself:
launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDF);
```

`PIDFCoefficients` and `setPIDFCoefficients` are the FTC SDK's own; there is no custom PIDF class.
**Live tuning:** change the numbers in FTC Dashboard (Configuration tab, under the subsystem's name)
while the robot runs; the motor gets them within one loop, no STOP and INIT needed. Graph the speed
against the target to see the effect: `launcher velocity` and `launcher target`, `intake velocity` and
`intake target`, and each wheel's `velocity` and `target`. When the graph looks good, **copy the numbers
into the `PIDF` line in the code**, because Dashboard changes are lost when the robot restarts.

**How to get your own numbers:**

1. Charge the battery. Put the robot on a block.
2. Run **BioBuzz Motor Max Speed**. Pick a motor with the D-pad and hold A for 3–4 seconds.
3. It shows the top speed and suggested numbers: `F = 32767 ÷ top speed`, `P = 0.1 × F`, `I = 0.1 × P`, `D = 0`, and a max speed of 90% of the top speed.
4. Copy them into that part's subsystem. Our drive motors are goBILDA **5203-2402-0019** (19.2:1, 312 RPM), counting 537.7 ticks per wheel turn: 537.7 × 312 ÷ 60 ≈ 2796 ticks/s top speed on a charged battery, which is why F starts at 32767 ÷ 2800 ≈ 11.7.
5. Drive and watch the velocity graphs on FTC Dashboard. Speed swings up and down: lower P. Speed stays below the target: raise P a little. Re-INIT after each change.

## Telemetry: FTC Dashboard and AdvantageScope

While `USE_DASHBOARD` is `true`, both programs send every telemetry line to the Driver Station
**and** to FTC Dashboard. With it `false`, the TeleOp sends none and the Auto sends its lines to the
Driver Station only.

1. Connect the laptop to the robot's Wi-Fi and open **http://192.168.43.1:8080/dash**.
2. Lines that are just a number (`launcher velocity`, `front left velocity`, `front left target`, `intake velocity`…) can be dragged into a **Graph** to watch them change.
3. The **Camera** view shows what the webcam sees.
4. The **Configuration** tab has one group per subsystem (`MecanumDriveSubsystem`, `IntakeSubsystem`, `LauncherSubsystem`, `AprilTagVision`). Change a number there while a program runs: a changed PIDF reaches the motors within one loop, with no rebuild and no INIT. Only change your own team's group. Copy the good values back into the code afterwards, because Dashboard changes are lost when the robot restarts.

**Safe tuning, one run at a time:** robot on a block (or lots of space), one hand on STOP. Run the same short test every time (for example full stick forward for 2 seconds), change **one** number, run again, and write each run in the notebook: run number, the numbers, what the graph did.

AdvantageScope can show the same live data, because it reads FTC Dashboard.

## Adding a step to the Auto

The Auto is a **state machine**: `AutonomousState` lists the steps (LOOK, AIM, SEARCH, LAUNCH, PARK, COMPLETE; AIM and SEARCH can switch back and forth),
and `loop()` runs the `case` for the current step until it says it is done. `nextStep(...)` moves to
the next step and restarts `stepTimer`. To slide 300 mm to the right after parking:

```java
private enum AutonomousState {
    AIM,
    SEARCH,
    LAUNCH,
    PARK,
    SLIDE,      // new
    COMPLETE
}
```

```java
case PARK:
    if (park()) {
        drive.stop();
        drive.resetEncoders();                // start the next move from 0
        nextStep(AutonomousState.SLIDE);
    }
    break;
case SLIDE:
    if (drive.strafeDistance(0.5, 300, DistanceUnit.MM, 1)) {   // + = right, - = left
        nextStep(AutonomousState.COMPLETE);
    }
    break;
```

**Always call `drive.resetEncoders()` between moves.** `driveDistance()`, `strafeDistance()` and
`rotateAngle()` aim at wheel positions counted from 0. Without the reset, the second move starts
from the wrong place. For a turn, use `drive.rotateAngle(0.2, 90, AngleUnit.DEGREES, 1)`:
positive = turn left.

## Test checklist after every change

1. **Build**: Android Studio → Build → Make Project. Fix any red errors first.
2. **Install** on the Control Hub and pick the OpMode on the Driver Station.
3. **During INIT**, check that telemetry shows the tags the camera sees.
4. **TeleOp on a block**: forward, back, strafe left/right, turn left/right, then hold the left bumper in front of a tag.
5. **Auto on the field**, with space around the robot and one hand on STOP.
6. **Watch telemetry**: `AutoState`, `Step time`, tags, motor positions vs. targets, launcher speed.
7. **Write down** what you changed and what happened.

## Ask a mentor before changing these

| What | Why |
|---|---|
| `TICKS_PER_REV` and `WHEEL_DIAMETER_MM` in `MecanumDriveSubsystem` | Only with new motors or wheels; every distance depends on them. |
| The launcher `PIDF` numbers | Tuned values; wrong numbers make the launcher speed swing or never reach 1200. |
| `double intakePower = 0;` at the top of the Auto's `loop()` | The intake only helps while the windmill feeds. |
| `isAtSpeed()` in `LauncherSubsystem` (speed > target − `FEED_MARGIN`) | Stops the windmill from feeding balls into a slow launcher. |
| `AprilTagVision.java` (the SDK's BIOBUZZ tag library, webcam name) | The SDK 12 library knows every HIVE tag's size and place, so it can measure to the CELL opening. A wrong name means no tags. |
| The method names every subsystem shares | The TeleOp and the Auto depend on them. |
| Anything with `hardwareMap.get(...)` | The name must match the Driver Hub configuration exactly. |

## Switching to a Limelight 3A (instead of the webcam)

The code is ready; nothing in the Auto or the TeleOp changes. `AprilTagVision` reads the Limelight
instead of the webcam and gives the same answers (which CELL is up, how far, at what angle).

1. **Plug it in** to a Control Hub USB port. In the Driver Hub configuration it shows up as a USB
   device: name it `limelight`. (You can leave `Webcam 1` in the configuration or remove it.)
2. **In the code:** `RobotSwitches.USE_LIMELIGHT = true` (keep `USE_CAMERA = true`), then rebuild.
3. **Run INIT and read the Driver Hub:** `Camera: Limelight 3A, pipeline 0, connected` and
   `Limelight tag size: ... mm`.
4. **In the Limelight's web page, pipeline 0:** AprilTag, family 36h11, that tag size, and full 3D on.
5. **Measure the new camera again:** `CAMERA_HEIGHT_IN` (lens height above the tiles) and
   `CAMERA_PITCH_DEG` (how far it tilts up) in `AprilTagVision`. These decide which CELL counts as UP.
6. **Check before a match:** face each CELL and read `Camera sees` and the UP/down lines. With the
   Limelight the tag log shows the real tag IDs, for example `RED AUDIENCE IDs 34 35 2/4 tags UP`.
7. **Re-save good launch poses** (operator B): the Limelight measures to the middle of the tags it
   sees, a few inches from where the webcam measured, so poses saved with the webcam won't match.

To go back to the webcam, set `USE_LIMELIGHT = false` and rebuild.

## Distance-based flywheel speed

A CELL far away needs a faster flywheel than a CELL close by. The launcher reads the distance to the
up CELL from the camera (webcam or Limelight) every loop and picks its speed from a straight line
through two measured points: **NEAR** (`NEAR_RANGE_IN`, `NEAR_VELOCITY`) and **FAR** (`FAR_RANGE_IN`,
`FAR_VELOCITY`) in `LauncherSubsystem`. Closer than NEAR it uses the NEAR speed, farther than FAR the
FAR speed. With no CELL in view it keeps its last speed. The Driver Hub shows
`Flywheel: FORWARD 1370 of 1381 ticks/s` and `Shot speed: for a CELL 60 in away, trim +0`.

**What the distance means.** It is the camera's AprilTag range: a straight line from the lens (12 in up)
to the center of the CELL opening (about 59.5 in up). So it is never below about 47.5 in, even right under
the HIVE. Along the floor: range 50.2 in ≈ 0.5 m, 54.4 in ≈ 0.75 m, 60.0 in ≈ 1.0 m, 66.6 in ≈ 1.25 m,
89.9 in ≈ 2 m, measured from the ball's release point (Runway v6: the front camera is 2 in ahead of it).

**Starting points (2026-10-20, Runway v6 front launch, not measured yet):** NEAR = 54.4 in / 1381 ticks/s and FAR = 89.9 in /
1994 ticks/s, from the 4-ball simulation (Solidworks-MCP repo, `DOCS/sim/hive_sim.py` on `volley_sim.py`: 77° exit toward the robot's FRONT from 19.2 cm, the CELL mouth's real lips;
launcher wheel + goBILDA 3628 flywheel, ball speed ≈ 0.375 × wheel tip speed, each ball's own speed drop
during the shot, air drag, aiming at the opening center, 151 cm). Earlier values 1375 / 1975 left out the
speed each ball takes from the wheel and would land ~5 cm low. The ball's center must pass 139.5–163 cm to go in, so aim at the
center, not the top edge. The model's speed scale is a guess, but the shape is not: the speed needed
rises almost in a straight line, about 15–18 ticks/s per inch of range. **Best shooting zone: range
52–67 in (0.6–1.25 m along the floor); best of all ≈ 54.4 in (0.75 m).** The up CELL's opening faces the
robot tipped 30° up (official field CAD), so a shot from closer than ~0.6 m is still rising too steeply
to go in; at 0.75 m the ball arrives falling ~40°, almost straight into the opening. Farther, the speed has to be held within about ±1%, which
`FEED_MARGIN` 25 does not guarantee. Details: Solidworks-MCP repo, `DOCS/Julia-Runway-Design.md`.

**Measuring NEAR and FAR (launcher team, practice field):**
1. Pick a close spot (about 54 in on the `Shot speed` line, about 0.75 m along the floor) and a far spot
   (about 90 in, about 2 m). Read the distance on the `Shot speed` line.
2. At the close spot, launch. If balls fall short, the operator presses **D-pad right** (+25 ticks/s);
   if they fly over, **D-pad left** (−25). When balls score, press **B**.
3. Do the same at the far spot.
4. Open `launch_log.csv`: each SUCCESS row has `tag_range_in` (the distance) and `launcher_tps` (the
   speed that scored). Put the close one in `NEAR_RANGE_IN` / `NEAR_VELOCITY` and the far one in
   `FAR_RANGE_IN` / `FAR_VELOCITY` (FTC Dashboard to try them, then the code to keep them).
5. Check a spot in between: it should score without trim. If not, measure a third distance and tell
   the programming team (a curve may be needed).

Set `USE_DISTANCE_SPEED = false` to always use `TARGET_VELOCITY`.

