# Code Review - 2026Rebuilt FRC Robot Project

## Executive Summary
This review covers the FRC robot codebase for the 2026Rebuilt project. The code implements a swerve drive robot with intake, shooter, kicker, and turret subsystems. Overall structure follows WPILib command-based architecture, but several critical issues need attention.

## Critical Issues

### 1. **Command Lifecycle Bug - Immediate Termination**
**Severity: High**

**Files Affected:**
- `ShootFuel.java` (line 68)
- `KickFuel.java` (line 33)
- `IntakeFuel.java` (line 34)
- `RotateTurret.java` (line 33)

**Issue:** All commands set `isDone = true` in their `execute()` method, causing them to terminate after a single scheduler cycle (~20ms). This prevents sustained operation.

**Impact:** Motors will only run for one cycle, making the robot non-functional.

**Recommendation:**
```java
// Remove isDone = true from execute() methods
// Only set isDone when the command should actually end
// For continuous commands, return false from isFinished()
```

### 2. **Static Subsystem References - Anti-Pattern**
**Severity: High**

**Files Affected:**
- `RobotContainer.java` (lines 58-60)
- All command classes

**Issue:** Subsystems declared as `public static` violates object-oriented principles and command-based framework design.

**Impact:**
- Prevents proper dependency injection
- Makes testing difficult
- Can cause initialization order issues
- Violates WPILib best practices

**Recommendation:**
```java
// Make subsystems instance variables
public final Intake intakeSystem;
public final Shoot shootSystem;
public final TurretControl turretControl;

// Pass subsystems to commands via constructor
new IntakeFuel(intakeSystem, speed);
```

### 3. **Unused Constructor Parameter**
**Severity: Medium**

**File:** `ShootFuel.java` (line 20)

**Issue:** The `shootSpeed` parameter is stored but the `speed` field is never used in `IntakeFuel.java`. The command always uses the constant from `Constants.Speeds.intakeMotorSpeed`.

**Recommendation:** Either use the parameter or remove it for consistency.

### 4. **Interpolation Map Initialization in Constructor**
**Severity: Low**

**File:** `ShootFuel.java` (lines 27-39)

**Issue:** The `kRegression` map is static but initialized in the constructor, causing redundant initialization for every command instance.

**Recommendation:**
```java
private static final InterpolatingDoubleTreeMap kRegression = new InterpolatingDoubleTreeMap();

static {
    kRegression.put(36.0, 2000.0);
    kRegression.put(72.0, 2500.0);
    // ... rest of values
}
```

## Code Quality Issues

### 5. **Inconsistent Parameter Types**
**Severity: Low**

**Files:** `IntakeFuel.java` vs `KickFuel.java`, `ShootFuel.java`

**Issue:** `IntakeFuel` uses `double speed` while `KickFuel` and `ShootFuel` use `int`. This inconsistency is confusing.

**Recommendation:** Standardize on `double` for all speed parameters.

### 6. **Commented-Out Code**
**Severity: Low**

**Files:** Multiple files contain extensive commented code

**Examples:**
- `ShootFuel.java` (lines 25, 63, 64, 72, 73)
- `KickFuel.java` (line 40)
- `RobotContainer.java` (lines 109-127)

**Recommendation:** Remove commented code or move to version control history. If needed for reference, document in comments why alternative approaches weren't used.

### 7. **Unused Imports and Variables**
**Severity: Low**

**File:** `Autos.java`

**Issue:** Several unused imports and commented-out code at the bottom.

**Recommendation:** Clean up unused imports and remove dead code.

### 8. **Magic Numbers**
**Severity: Medium**

**Files:** `TurretControl.java`, `RobotContainer.java`

**Examples:**
- `0.48`, `0.47`, `0.49` (rotation limits)
- `0.26` (search speed)
- `0.75` (max speed multiplier)
- `0.1` (deadband multiplier)

**Recommendation:** Extract to named constants with descriptive names.

### 9. **Kicker Speed Parameter Ignored**
**Severity: Medium**

**File:** `Shoot.java` (line 79)

**Issue:** `setKickerSpeed()` accepts a parameter but always uses `Constants.Speeds.kickMotorSpeed`.

```java
public void setKickerSpeed(int kickerVelocity) {
    m_kicker_controller.setSetpoint(Constants.Speeds.kickMotorSpeed, ControlType.kVelocity);
    // Should use kickerVelocity parameter
}
```

**Recommendation:** Use the parameter or remove it.

## Design Concerns

### 10. **Winch Motor Configuration Duplication**
**Severity: Low**

**File:** `Intake.java` (lines 59-75)

**Issue:** `releaseWinchMotors()` and `brakeWinchMotors()` contain nearly identical code with only the idle mode differing.

**Recommendation:**
```java
private void setWinchIdleMode(IdleMode mode) {
    winchMotorLeftConfig.idleMode(mode);
    winchMotorLeft.configureAsync(winchMotorLeftConfig, ...);
    winchMotorRightConfig.idleMode(mode);
    winchMotorRightConfig.inverted(true);
    winchMotorRightConfig.follow(winchMotorLeft);
    winchMotorRight.configureAsync(winchMotorRightConfig, ...);
}

public void releaseWinchMotors() { setWinchIdleMode(IdleMode.kCoast); }
public void brakeWinchMotors() { setWinchIdleMode(IdleMode.kBrake); }
```

### 11. **Encoder Position Check Logic**
**Severity: Medium**

**File:** `Intake.java` (line 79)

**Issue:** `retractIntake()` checks if position > 10, but the logic seems inverted. If retracting, you'd expect to stop when position reaches a minimum, not continue while above a threshold.

**Recommendation:** Verify the intended behavior and add comments explaining the logic.

### 12. **Search Direction Logic Bug**
**Severity: High**

**File:** `TurretControl.java` (lines 197-199)

**Issue:** The search direction logic has inverted conditions:
```java
if (currentRotation >= 0.47) {
    searchDirectionRight = false;
} else if (currentRotation <= 0.49) {  // This will always be true if first condition is false
    searchDirectionRight = true;
}
```

**Recommendation:**
```java
if (currentRotation >= 0.47) {
    searchDirectionRight = false;
} else if (currentRotation <= -0.47) {
    searchDirectionRight = true;
}
```

### 13. **Missing Null Safety**
**Severity: Medium**

**File:** `TurretControl.java`

**Issue:** `tv` (target visible) is a `Boolean` object, not primitive `boolean`. This can be null, causing NullPointerException.

**Recommendation:**
```java
public boolean isTargetVisible() {
    return tv != null && tv;
}
```

## Best Practices Recommendations

### 14. **Command Factories**
Multiple files have the comment suggesting Command factories API. This is valid advice.

**Recommendation:** Consider refactoring simple commands to use Command factories:
```java
// Instead of IntakeFuel command class
Commands.startEnd(
    () -> intakeSystem.runIntake(),
    () -> intakeSystem.stopIntake(),
    intakeSystem
);
```

### 15. **PID Tuning Values**
**File:** `Shoot.java`, `TurretControl.java`

**Issue:** PID constants are hardcoded. Consider using SmartDashboard for tuning.

**Recommendation:**
```java
SmartDashboard.putNumber("Shooter kP", kPshoot);
// Read back and apply during periodic
```

### 16. **Error Handling**
**Severity:** Medium

**Issue:** No error handling for motor controller configuration failures or Limelight communication issues.

**Recommendation:** Add error checking and logging for hardware initialization and communication.

### 17. **Documentation**
**Severity:** Low

**Issue:** Minimal JavaDoc comments. Many methods lack documentation.

**Recommendation:** Add JavaDoc comments explaining:
- Method purpose
- Parameter meanings
- Return value descriptions
- Units of measurement

## Security & Safety

### 18. **Soft Limits**
**File:** `TurretControl.java`

**Issue:** Soft limits are set to ±2 radians (~115°) but search logic uses ±0.47 rotations (~170°). Inconsistent units and values.

**Recommendation:** Verify mechanical limits and ensure software limits match. Use consistent units throughout.

### 19. **Motor Safety**
**Issue:** No current limiting or thermal protection explicitly configured.

**Recommendation:** Review motor controller configurations for current limits appropriate to your motors.

## Testing Recommendations

1. **Unit Tests:** Add unit tests for:
   - Distance calculations in `TurretControl`
   - Interpolation logic in `ShootFuel`
   - State machine transitions

2. **Simulation:** Leverage WPILib simulation to test:
   - Command sequences
   - Autonomous routines
   - Turret tracking logic

3. **Integration Tests:** Test full command sequences:
   - Intake → Shoot → Kick sequence
   - Turret tracking → Shoot sequence

## Priority Action Items

1. **CRITICAL:** Fix command lifecycle bugs (Issue #1)
2. **CRITICAL:** Remove static subsystem references (Issue #2)
3. **HIGH:** Fix turret search direction logic (Issue #12)
4. **HIGH:** Fix kicker speed parameter usage (Issue #9)
5. **MEDIUM:** Add null safety for Limelight data (Issue #13)
6. **MEDIUM:** Verify and fix winch retraction logic (Issue #11)
7. **LOW:** Clean up commented code and extract magic numbers

## Positive Aspects

- Good use of WPILib 2025 features (Units API, Choreo integration)
- Proper use of closed-loop control for shooters
- State machine implementation for turret control
- Integration with Limelight for vision tracking
- Swerve drive implementation using CTRE Phoenix 6

## Conclusion

The codebase demonstrates good understanding of FRC programming concepts but requires immediate attention to critical bugs before deployment. Focus on fixing command lifecycle issues and refactoring static subsystem references first, then address the remaining issues in priority order.
