# DECODE Autonomous Strategy

## Overview of DECODE scoring

DECODE is an FTC game where robots collect **Artifacts** and place them into a **Classifier/Goal** for points. Autonomous points are typically higher than Driver-Controlled, and there are additional bonuses for leaving the launch line and parking in the base zone.

**Key scoring concepts your auto should aim for:**

- **Leave the Launch Line** early (3 points)
- **Collect and score artifacts** consistently (3 points each for classified, 1 for overflow)
- **Match the motif pattern** (2 points per correct color match)
- **Finish by parking in the Base Zone** (5-10 points, plus 10 bonus if both robots park)

## Current Autonomous Implementation

### Available OpModes

| OpMode | Alliance | Description |
|--------|----------|-------------|
| `Auto 3 Artifacts BLUE` | Blue | Time-based autonomous for blue alliance |
| `Auto 3 Artifacts RED` | Red | Mirrored version for red alliance |
| `Encoder Auto Blue Obelisk` | Blue | Encoder-based autonomous (currently disabled) |

### Code Architecture

```
BaseAutoOpMode (abstract)
├── AutoThreeArtifacts (Blue alliance)
└── AutoThreeArtifactsRed (Red alliance)

Hardware.RobotHardware
├── Motors: lf, lr, rf, rr (drive), i (intake), sf1/sf2 (shooter), turret
└── Used by: TeleOpFinal, BaseAutoOpMode, EncoderAutoBlueObelisk
```

### Hardware Configuration Names

The robot uses these configuration names (set in Robot Controller app):

| Config Name | Hardware | Description |
|-------------|----------|-------------|
| `lf` | DcMotorEx | Left front drive motor |
| `lr` | DcMotorEx | Left rear drive motor |
| `rf` | DcMotorEx | Right front drive motor |
| `rr` | DcMotorEx | Right rear drive motor |
| `i` | DcMotorEx | Intake motor |
| `sf1` | DcMotorEx | Shooter flywheel 1 |
| `sf2` | DcMotorEx | Shooter flywheel 2 |
| `turret` | DcMotorEx | Turret rotation motor |
| `webcam` | WebcamName | AprilTag camera |

## "Three Artifacts" Autonomous Routine

The autonomous routine follows these steps:

1. **Leave Launch Line** (800ms forward at 50% power)
   - Scores 3 points immediately

2. **Collect Artifacts** (2000ms intake while driving slowly)
   - Intake motor runs at 100% power
   - Robot drives slowly forward (20% power) to gather artifacts

3. **Navigate to Classifier**
   - Strafe toward goal (900ms at 50% power)
   - Blue = strafe RIGHT, Red = strafe LEFT
   - Drive forward to approach (700ms at 40% power)

4. **Deposit Artifacts** (3000ms)
   - Shooter flywheels spin up (500ms warmup)
   - Intake reverses to feed artifacts into shooter
   - Artifacts launched into classifier

5. **Return to Base** (1200ms backward at 50% power)
   - Scores 5-10 points for parking

### Safety Features

- **30-second timeout**: All movements check `shouldStop()` which returns true at 29.5 seconds
- **Stop on request**: Movements also check `isStopRequested()` for user interruption
- **Motor stop**: `stopAllMotors()` ensures clean shutdown

## Tuning the Autonomous

### Using FTC Dashboard

All timing and power values can be adjusted live via FTC Dashboard:

1. Connect to robot WiFi
2. Open browser to `http://192.168.43.1:8080/dash`
3. Find `AutoThreeArtifacts` or `AutoThreeArtifactsRed` in the configuration panel
4. Adjust values and re-run autonomous

### Tunable Parameters

```java
// Step 1: Leave launch line
LEAVE_LINE_POWER = 0.5      // Motor power (0.0-1.0)
LEAVE_LINE_MS = 800         // Duration in milliseconds

// Step 2: Collect artifacts
INTAKE_POWER = 1.0          // Intake motor power
INTAKE_MS = 2000            // Duration to run intake
INTAKE_DRIVE_POWER = 0.2    // Forward drive while intaking

// Step 3: Navigate to classifier
STRAFE_POWER = 0.5          // Strafe movement power
STRAFE_MS = 900             // Strafe duration
APPROACH_POWER = 0.4        // Forward approach power
APPROACH_MS = 700           // Approach duration

// Step 4: Deposit artifacts
SHOOTER_POWER = 1.0         // Flywheel power
FEED_POWER = -1.0           // Intake reverse to feed (negative)
DEPOSIT_MS = 3000           // Total deposit time

// Step 5: Return to base
RETURN_POWER = 0.5          // Backward drive power
RETURN_MS = 1200            // Return duration
```

### Tuning Process

1. **Start with leave line**: Adjust `LEAVE_LINE_MS` until robot clears the launch line
2. **Tune intake**: Test `INTAKE_MS` to ensure 3 artifacts are collected
3. **Calibrate navigation**:
   - Adjust `STRAFE_MS` to align with classifier horizontally
   - Adjust `APPROACH_MS` to reach scoring distance
4. **Test deposit**: Verify `SHOOTER_POWER` and `DEPOSIT_MS` launch all 3 artifacts
5. **Check return**: Ensure robot returns to base within 30 seconds

## Mirroring for Red vs Blue

The field is mirrored, so Red and Blue have identical code except for strafe direction:

| Alliance | Strafe Direction | Constant Value |
|----------|------------------|----------------|
| Blue | RIGHT | `STRAFE_DIRECTION = 1` |
| Red | LEFT | `STRAFE_DIRECTION = -1` |

## Upgrading to Encoder-Based Movement

The `BaseAutoOpMode` includes encoder-based methods for more consistent movements:

```java
// Instead of time-based:
moveForTime(0.5, 0.5, 800);

// Use encoder-based:
driveDistance(12, 0.5);  // 12 inches forward at 50% power
strafeDistance(8, 0.5);  // 8 inches right at 50% power
turnDegrees(90, 0.5);    // 90 degrees right at 50% power
```

### When to Switch to Encoders

Switch after the time-based version works reliably. Benefits:
- Consistent distances regardless of battery voltage
- More repeatable autonomous
- Easier to understand (inches vs milliseconds)

## Troubleshooting

### Robot doesn't move
- Check motor configuration names match hardware map
- Verify motors are plugged in correctly
- Check battery voltage

### Robot goes wrong direction
- Motor directions may need reversing in `Hardware/RobotHardware.java`
- Strafe direction might be inverted for your motor configuration

### Robot doesn't score artifacts
- Increase `SHOOTER_POWER` or `DEPOSIT_MS`
- Check shooter flywheel direction
- Verify `FEED_POWER` is negative (reverse intake)

### Robot runs past 30 seconds
- All movements now include `shouldStop()` check
- Reduce total timing if running over

### Encoders aren't working
- Ensure `USING_ODOMETRY` is `false` in `Globals.java`
- Check encoder cables are connected
- Verify `resetEnc()` is called before movement

## Next Steps After Basic Auto Works

1. **Add encoder-based movements** for consistency
2. **Add IMU heading correction** for straight driving
3. **Implement pattern matching** using camera to detect motif
4. **Add second scoring cycle** if time permits
5. **Optimize timing** to maximize points within 30 seconds

## Competition Checklist

Before each match:
- [ ] Select correct alliance color OpMode (BLUE or RED)
- [ ] Verify robot is positioned correctly on launch line
- [ ] Check battery voltage is adequate (>12V)
- [ ] Confirm intake and shooter mechanisms are clear
- [ ] Test-run autonomous on practice field if time permits
