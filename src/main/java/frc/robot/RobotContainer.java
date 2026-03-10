// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.DriveIntakeOut;
import frc.robot.commands.IntakeFuel;
import frc.robot.commands.KickFuel;
import frc.robot.commands.ReleaseIntake;
import frc.robot.commands.RetractIntake;
import frc.robot.commands.RotateTurret;
import frc.robot.commands.ShootFuel;
import frc.robot.commands.StopWinch;
import frc.robot.commands.UnjamFuel;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shoot;
import frc.robot.subsystems.TurretControl;

public class RobotContainer {
        private double MaxSpeed = 0.75 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired
                                                                                             // top
                                                                                             // speed
        private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                          // second
                                                                                          // max angular velocity

        /* Setting up bindings for necessary control of the swerve drive platform */
        public final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                                 // motors

        public final SwerveRequest.RobotCentric rcdrive = new SwerveRequest.RobotCentric()
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                                 // motors

        // private final SwerveRequest.SwerveDriveBrake brake = new
        // SwerveRequest.SwerveDriveBrake();
        // private final SwerveRequest.PointWheelsAt point = new
        // SwerveRequest.PointWheelsAt();

        private final Telemetry logger = new Telemetry(MaxSpeed);

        private final CommandXboxController driver = new CommandXboxController(0);
        private final CommandXboxController codriver = new CommandXboxController(1);

        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

        public static Intake intakeSystem;
        public static Shoot shootSystem;
        public static TurretControl turretControl;

        private boolean isTurretTracking = true;

        public RobotContainer() {
                intakeSystem = new Intake();
                shootSystem = new Shoot();
                turretControl = new TurretControl();
                configureBindings();
        }

        private void configureBindings() {
                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.
                drivetrain.setDefaultCommand(
                                // Drivetrain will execute this command periodically
                                drivetrain.applyRequest(() -> drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive
                                                                                                                 // forward
                                                                                                                 // with
                                                                                                                 // negative
                                                                                                                 // Y
                                                                                                                 // (forward)
                                                .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with
                                                                                              // negative X (left)
                                                .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive
                                                                                                          // counterclockwise
                                                                                                          // with
                                                                                                          // negative X
                                                                                                          // (left)
                                ));

                // Idle while the robot is disabled. This ensures the configured
                // neutral mode is applied to the drive motors while disabled.
                final var idle = new SwerveRequest.Idle();
                RobotModeTriggers.disabled().whileTrue(
                                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

                // driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
                // driver.b().whileTrue(drivetrain
                // .applyRequest(() -> point.withModuleDirection(new
                // Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

                // Reset the field-centric heading on left bumper press.
                driver.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

                // Intake
                driver.rightTrigger(0.1).whileTrue(
                                new IntakeFuel(Constants.Speeds.intakeMotorSpeed)).onFalse(new IntakeFuel(0));

                // Shooter

                driver.y().onTrue(Commands.sequence(
                                new ShootFuel(Constants.Speeds.shootMotorSpeed),
                                Commands.waitSeconds(0.5),
                                new KickFuel(Constants.Speeds.kickMotorSpeed)))
                                .onFalse(Commands.sequence(new KickFuel(0), new ShootFuel(0)));

                /*
                 * driver.y().whileTrue(new
                 * ShootFuel(Constants.Speeds.shootMotorSpeed)).onFalse(new ShootFuel(0));
                 */
                driver.a().whileTrue(new RetractIntake(100)).onFalse(new StopWinch());
                driver.b().whileTrue(new ReleaseIntake()).onFalse(new StopWinch());
                driver.x().whileTrue(new DriveIntakeOut(100)).onFalse(new StopWinch());

                // codriver.leftTrigger(0.1).whileTrue(new
                // RotateTurret(-codriver.getLeftTriggerAxis()))
                // .onFalse(new RotateTurret(0));
                // codriver.rightTrigger(0.1).whileTrue(new
                // RotateTurret(codriver.getRightTriggerAxis()))
                // .onFalse(new RotateTurret(0));

                codriver.leftTrigger(0.1).whileTrue(
                                Commands.repeatingSequence(
                                                turretControl.setState(Constants.turretStates.MANUAL),
                                                new InstantCommand(() -> {
                                                        turretControl.driveTurret(-codriver.getLeftTriggerAxis());
                                                })))
                                .onFalse(new RotateTurret(0));

                codriver.rightTrigger(0.1).whileTrue(
                                Commands.repeatingSequence(
                                                turretControl.setState(Constants.turretStates.MANUAL),
                                                new InstantCommand(() -> {
                                                        turretControl.driveTurret(codriver.getRightTriggerAxis());
                                                })))
                                .onFalse(new RotateTurret(0));

                codriver.b().onTrue(Commands.runOnce(() -> {
                        if (isTurretTracking) {
                                turretControl.setStateDirectly(Constants.turretStates.HOME);
                                isTurretTracking = false;
                        } else {
                                turretControl.setStateDirectly(Constants.turretStates.TRACKING);
                                isTurretTracking = true;
                        }
                }));

                codriver.y().onTrue(Commands.sequence(
                                new ShootFuel(Constants.Speeds.shootMotorSpeed),
                                Commands.waitSeconds(0.5),
                                new KickFuel(Constants.Speeds.kickMotorSpeed)))
                                .onFalse(Commands.sequence(new KickFuel(0), new ShootFuel(0)));

                codriver.x().onTrue(
                                new UnjamFuel())
                                .onFalse(Commands.sequence(new KickFuel(0)));

                drivetrain.registerTelemetry(logger::telemeterize);
        }

}
