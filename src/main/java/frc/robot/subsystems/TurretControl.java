package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.units.Units;
import frc.robot.LimelightHelpers;
import frc.robot.Utils;
import frc.robot.Constants;

public class TurretControl extends SubsystemBase {

    // these are the AprilTags on the hubs that we'll target; we'll ignore any others when shooting
    private final int[] redAprilTags = {8, 10, 11};
    private final int[] blueAprilTags = {18, 24, 26, 27};

    private final SparkMax turretMotor;
    private final RelativeEncoder turretEncoder;
    SparkClosedLoopController m_controller;

    private final double forwardSoftLimit = 0.2; // max angle in radians
    private final double reverseSoftLimit = -0.2; // min angle in radians

    private final double gearRatio = 90;
    private final double kP = 0.2;
    private final double kI = 0;
    private final double kD = 0;
    private final double kS = 0.1;

    // PID for running turret to angle position
    private final double kP1 = 0.000001;
    private final double kI1 = 0;
    private final double kD1 = 0.05;
    private final double kS1 = 0.0;

    private Constants.turretStates currentState;
    private Angle tx, ty;
    private Boolean tv;
    private Boolean searchDirectionRight;

    public TurretControl() {
        currentState = Constants.turretStates.DEFAULT;
        turretMotor = new SparkMax(Constants.Motors.turretMotorID, MotorType.kBrushless);
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.idleMode(IdleMode.kBrake);
        turretEncoder = turretMotor.getEncoder();
        m_controller = turretMotor.getClosedLoopController();

        motorConfig.softLimit
            .forwardSoftLimit(forwardSoftLimit)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(reverseSoftLimit)
            .reverseSoftLimitEnabled(true);

        motorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(kP, kI, kD, ClosedLoopSlot.kSlot0)
            .pid(kP1, kI1, kD1, ClosedLoopSlot.kSlot1)
            .feedForward.kS(kS, ClosedLoopSlot.kSlot0).kS(kS1, ClosedLoopSlot.kSlot1);

        // Configure Encoder Gear Ratio
        motorConfig.encoder
            .positionConversionFactor(1 / gearRatio)
            .velocityConversionFactor((1 / gearRatio) / 60); // Covnert RPM to RPS

        turretMotor.configure(
            motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters
        );
        
        // Reset encoder position to zero
        turretEncoder.setPosition(0);
    }

    @Override
    public void periodic() {
        super.periodic();

        /*
         * Can set via code which AprilTags to look for using
         * LimelightHelpers.SetFiducialIDFiltersOverride("limelight", [1, 2, 3]);
         * We could switch based on the Alliance we're on
         */
        if (Utils.isRed()) {
            LimelightHelpers.SetFiducialIDFiltersOverride("", redAprilTags);
        } else {
            LimelightHelpers.SetFiducialIDFiltersOverride("", blueAprilTags);
        }

        tx = Units.Degrees.of(LimelightHelpers.getTX(""));
        ty = Units.Degrees.of(LimelightHelpers.getTY(""));
        tv = LimelightHelpers.getTV("");

        // This method will be called once per scheduler run
        // DEFAULT, HOME, TRACKING, SEARCHING, MANUAL
        switch (currentState) {
            case HOME:
                goHome();
                break;
            case TRACKING:
                trackTarget();
                // if (!isTargetVisible()) {
                //     currentState = Constants.turretStates.SEARCHING;
                // }
                break;
            case SEARCHING:
                // findTarget();
                if (isTargetVisible()) {
                    currentState = Constants.turretStates.TRACKING;
                }
                break;
            case MANUAL:
            // Do nothing here, we'll be calling setVelocity from RobotContainer
            // when in MANUAL mode
                break;
            default:
                stopMotor();
        }
        SmartDashboard.putString("Turret Mode", currentState.name());
        SmartDashboard.putBoolean("Target Visible", tv);
        SmartDashboard.putNumber("TurretLimit", turretEncoder.getPosition());
    }

    public Command setState(Constants.turretStates state) {
        return new InstantCommand(() -> currentState = state, this);
    }

    public void setStateDirectly(Constants.turretStates state) {
        this.currentState = state;
    }

    public double getDistanceAsDouble() {
        double distance = 0;
        if (isTargetVisible()) {
            // TARGET_HEIGHT needs to be defined as a Distance (in Inch)
            // Distance TARGET_HEIGHT = Distance.ofBaseUnits(36, Inch)
            Distance opposite = Constants.Dimensions.targetHeight.minus(Constants.Dimensions.limelightHeight);
            // ty is from the limelight helper getTy()
            Angle theta = ty.plus(Constants.Dimensions.limelightMountingAngle);
            distance = opposite.in(Units.Inches) / Math.tan(theta.in(Units.Radians));
        }
        SmartDashboard.putNumber("Hub Distance", distance);
        return distance;
    }
    public Distance getDistance() {
        double distance = getDistanceAsDouble();
        return Distance.ofBaseUnits(distance, Units.Inches);
    }

    public Angle getTargetAngle() {
        // get the angle to the target relative to where the turret
        // is currently pointing. This is how far we'll have to turn
        // We'd get this from the Limelight's tx value
        Distance distance = getDistance();
        Distance oppposite = distance.times(Math.tan(tx.in(Units.Radians))).plus(Constants.Dimensions.limelightXOffset);
        Distance adjacent = distance.plus(Constants.Dimensions.limelightYOffset);
        return Units.Radians.of(Math.atan(oppposite.in(Units.Inches) / adjacent.in(Units.Inches)));
    }

    public Angle getTurretFacingAngle() {
        // get the current angle the turret is turned to, would be
        // based on the encoder's absolute position with zero being
        // straight ahead
        return Angle.ofBaseUnits(turretEncoder.getPosition(), Rotations);
    }

    public boolean isTargetVisible() {
        return tv;
    }

    private void trackTarget() {
        // uses PID to drive tx to 0
        double power = 0;
        Angle currentAngle = getTurretFacingAngle();
        Angle offsetAngle = getTargetAngle();
        Angle targetAngle = currentAngle.plus(offsetAngle);
        Angle clampedAngle = Units.Rotations.of(Math.max(-0.48, Math.min(0.48, targetAngle.in(Rotations))));
        SmartDashboard.putNumber("ClampedAngle", clampedAngle.baseUnitMagnitude());
        // Set the setpoint of the PID controller in raw position mode
        //m_controller.setSetpoint(clampedAngle.baseUnitMagnitude(), ControlType.kPosition, ClosedLoopSlot.kSlot1);
        if(Math.abs(clampedAngle.baseUnitMagnitude()) > 0.02){
            power = 0.85 * clampedAngle.baseUnitMagnitude(); //(0.5 * (1 - (targetAngle.div(offsetAngle)).baseUnitMagnitude()));
        }
        else {
            power = 0;
        }
        m_controller.setSetpoint(power, ControlType.kDutyCycle, ClosedLoopSlot.kSlot1);
    }

    private void findTarget() {
        double currentRotation = getTurretFacingAngle().in(Units.Rotations);
        SmartDashboard.putNumber("current rotation", currentRotation);
        double searchSpeed = 0.26; // radians/sec
        if (currentRotation >= 0.35) {
            searchDirectionRight = true;
        } else if (currentRotation < 0.35) {
            searchDirectionRight = false;
        }

        double finalVelocity = searchDirectionRight ? searchSpeed : -searchSpeed;
        m_controller.setSetpoint(finalVelocity, ControlType.kDutyCycle);
        SmartDashboard.putNumber("finalVelocity", finalVelocity);
                                                                                    // deg/sec
    }

    public void goHome() {
        m_controller.setSetpoint(0, ControlType.kPosition);
    }

    public void stopMotor() {
        turretMotor.stopMotor();
        // turretMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    /*
    * Manually drives the turret at the speed specified
    * @param speed Trigger input in range from +- 0 to 1
     */    
    public void driveTurret (double speed) {
        double currentRotation = getTurretFacingAngle().in(Units.Rotations);
        int motorSpeed = (int) (Constants.Speeds.neoRPM * Constants.Speeds.turretMotorFactor * speed);
        SmartDashboard.putNumber("Turret Rotate Speed", motorSpeed);
        if (currentRotation <= 0.46 && currentRotation >=-0.46){
            //m_controller.setSetpoint (motorSpeed, ControlType.kVelocity);
            m_controller.setSetpoint(speed * 0.25, ControlType.kDutyCycle, ClosedLoopSlot.kSlot0);
        }
    }
}
