package frc.robot;

import java.time.Instant;
import java.util.Optional;

import choreo.auto.AutoFactory;
import frc.robot.commands.DriveIntakeOut;
import frc.robot.commands.KickFuel;
import frc.robot.commands.ShootFuel;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

public class Autos {
        private final CommandSwerveDrivetrain drivetrain;
        private final RobotContainer robotContainer;
        private final Routines routines;
        private final PIDController autoPIDx;
        private final PIDController autoPIDy;
        private final PIDController autoPIDangular;
        private static final double kAutoKp = 7.0;
        private static final double kAutoKi = 0.0;
        private static final double kAutoKd = 0.0;

        private static final double kAutoAngularKp = 5.0;
        private static final double kAutoAngularKi = 0.0;
        private static final double kAutoAngularKd = 0.0;

        private final AutoFactory autoFactory;

        public Autos(Robot robot) {
                drivetrain = robot.drivetrain;
                robotContainer = robot.m_robotContainer;
                routines = robot.routines;
                autoPIDx = new PIDController(kAutoKp, kAutoKi, kAutoKd);
                autoPIDy = new PIDController(kAutoKp, kAutoKi, kAutoKd);
                autoPIDangular = new PIDController(kAutoAngularKp, kAutoAngularKi, kAutoAngularKd);
                autoPIDangular.enableContinuousInput(-Math.PI, Math.PI);

                // Create the auto factory - Need to look at how this needs to be implemented
                // connecting CTRE with Choreo
                autoFactory = drivetrain.createAutoFactory();

        }

        public Command drive_Out_RightCommand() {
                // create a new Routine with the name "Drive8_Test" - which should match the
                // trajectory you'll create next
                AutoRoutine routine = autoFactory.newRoutine("Drive_Out_Right");
                // Read in the trajectory named "LeaveHome" - this has to be the case-sensitive
                // name from Choreo
                AutoTrajectory drive_Out_RightTrajectory = routine.trajectory("Drive_Out_Right");
                // when the routine is active (auto is enabled) run a sequence of commands -
                // print a message, reset odometry,
                // then drive our trajectory
                routine.active()
                                .onTrue(Commands.sequence(Commands.print("Drive_Out_Right"),
                                                drive_Out_RightTrajectory.resetOdometry(),
                                                drive_Out_RightTrajectory.cmd()));
                // finally, when that trajectory is done, stop the bot using the routine from
                // Routines.java
                drive_Out_RightTrajectory.done().onTrue(Commands.sequence(
                                new ShootFuel(Constants.Speeds.shootMotorSpeed)
                                                .andThen(
                                                                Commands.parallel(
                                                                                robotContainer.turretControl.setState(
                                                                                                Constants.turretStates.TRACKING),
                                                                                Commands.race(new DriveIntakeOut(100),
                                                                                Commands.waitSeconds(0.5)),
                                                                                Commands.race(new KickFuel(
                                                                                                Constants.Speeds.kickMotorSpeed)),
                                                                                Commands.waitSeconds(10)))
                                                .andThen(
                                                                Commands.parallel(
                                                                                Commands.race(
                                                                                                new ShootFuel(0),
                                                                                                Commands.waitSeconds(
                                                                                                                .5)),
                                                                                Commands.race(
                                                                                                new DriveIntakeOut(100),
                                                                                                Commands.waitSeconds(1)))),
                                robotContainer.turretControl.setState(Constants.turretStates.MANUAL)
                                                .andThen(Commands.sequence(
                                                                new ShootFuel(0))),
                                                                new KickFuel(0)
                                                .andThen(
                                                                routines.stopBot())));

                return routine.cmd();
        }

        public Command drive_Out_LeftCommand() {
                // create a new Routine with the name "Drive8_Test" - which should match the
                // trajectory you'll create next
                AutoRoutine routine = autoFactory.newRoutine("Drive_Out_Left");
                // Read in the trajectory named "LeaveHome" - this has to be the case-sensitive
                // name from Choreo
                AutoTrajectory drive_Out_LeftTrajectory = routine.trajectory("Drive_Out_Left");
                // when the routine is active (auto is enabled) run a sequence of commands -
                // print a message, reset odometry,
                // then drive our trajectory
                routine.active()
                                .onTrue(Commands.sequence(Commands.print("Drive_Out_Left"),
                                                drive_Out_LeftTrajectory.resetOdometry(),
                                                drive_Out_LeftTrajectory.cmd()));
                // finally, when that trajectory is done, stop the bot using the routine from
                // Routines.java
                drive_Out_LeftTrajectory.done().onTrue(Commands.sequence(
                                new ShootFuel(Constants.Speeds.shootMotorSpeed)
                                                .andThen(
                                                                Commands.parallel(
                                                                                robotContainer.turretControl.setState(
                                                                                                Constants.turretStates.TRACKING),
                                                                                Commands.race(new DriveIntakeOut(100),
                                                                                Commands.waitSeconds(0.5)),
                                                                                Commands.race(new KickFuel(
                                                                                                Constants.Speeds.kickMotorSpeed)),
                                                                                Commands.waitSeconds(10)))
                                                .andThen(
                                                                Commands.parallel(
                                                                                Commands.race(
                                                                                                new ShootFuel(0),
                                                                                                Commands.waitSeconds(
                                                                                                                .5)),
                                                                                Commands.race(
                                                                                                new DriveIntakeOut(100),
                                                                                                Commands.waitSeconds(1)))),
                                robotContainer.turretControl.setState(Constants.turretStates.MANUAL)
                                                .andThen(Commands.sequence(
                                                                new ShootFuel(0))),
                                                                new KickFuel(0)
                                                .andThen(
                                                                routines.stopBot())));

                return routine.cmd();
        }

        public Command drive_Out_MiddleCommand() {
                // create a new Routine with the name "Drive8_Test" - which should match the
                // trajectory you'll create next
                AutoRoutine routine = autoFactory.newRoutine("Drive_Out_Middle");
                // Read in the trajectory named "LeaveHome" - this has to be the case-sensitive
                // name from Choreo
                AutoTrajectory drive_Out_MiddleTrajectory = routine.trajectory("Drive_Out_Middle");
                // when the routine is active (auto is enabled) run a sequence of commands -
                // print a message, reset odometry,
                // then drive our trajectory
                routine.active()
                                .onTrue(Commands.sequence(Commands.print("Drive_Out_Middle"),
                                                drive_Out_MiddleTrajectory.resetOdometry(),
                                                drive_Out_MiddleTrajectory.cmd()));
                // finally, when that trajectory is done, stop the bot using the routine from
                // Routines.java
                drive_Out_MiddleTrajectory.done().onTrue(Commands.sequence(
                                new ShootFuel(Constants.Speeds.shootMotorSpeed)
                                                .andThen(
                                                                Commands.parallel(
                                                                                robotContainer.turretControl.setState(
                                                                                                Constants.turretStates.TRACKING),
                                                                                Commands.race(new DriveIntakeOut(100),
                                                                                Commands.waitSeconds(0.5)),
                                                                                Commands.race(new KickFuel(
                                                                                                Constants.Speeds.kickMotorSpeed)),
                                                                                Commands.waitSeconds(10)))
                                                .andThen(
                                                                Commands.parallel(
                                                                                Commands.race(
                                                                                                new ShootFuel(0),
                                                                                                Commands.waitSeconds(
                                                                                                                .5)),
                                                                                Commands.race(
                                                                                                new DriveIntakeOut(100),
                                                                                                Commands.waitSeconds(1)))),
                                robotContainer.turretControl.setState(Constants.turretStates.MANUAL)
                                                .andThen(Commands.sequence(
                                                                new ShootFuel(0))),
                                                                new KickFuel(0)
                                                .andThen(
                                                                routines.stopBot())));

                return routine.cmd();
        }

        public Command ScorePre_GoDepotCommand() {
                // create a new Routine with the name "Drive8_Test" - which should match the
                // trajectory you'll create next
                AutoRoutine routine = autoFactory.newRoutine("ScorePre_GoDepot");
                // Read in the trajectory named "LeaveHome" - this has to be the case-sensitive
                // name from Choreo
                AutoTrajectory ScorePre_GoDepotTrajectory = routine.trajectory("ScorePre_GoDepot");
                // when the routine is active (auto is enabled) run a sequence of commands -
                // print a message, reset odometry,
                // then drive our trajectory
                routine.active()
                                .onTrue(Commands.sequence(Commands.print("running ScorePre_GoDepot"),
                                                ScorePre_GoDepotTrajectory.resetOdometry(),
                                                ScorePre_GoDepotTrajectory.cmd()));
                // finally, when that trajectory is done, stop the bot using the routine from
                // Routines.java
                ScorePre_GoDepotTrajectory.done()
                                .onTrue(Commands.sequence(Commands.print("Auto Completed"), routines.stopBot()));
                return routine.cmd();
        }
        // public void driveRobotCentric(SwerveSample sample) {
        // Pose2d pose = drivetrain.getPose();
        // Commands.print("vx" + sample.vx);
        // Commands.print("vy" + sample.vy);
        // Commands.print("x" + sample.x);
        // Commands.print("y" + sample.y);
        // drivetrain.applyRequest(() ->
        // robotContainer.rcdrive
        // .withVelocityX(sample.vx + autoPIDx.calculate(pose.getX(), sample.x))
        // .withVelocityY(sample.vy + autoPIDy.calculate(pose.getY(), sample.y))
        // .withRotationalRate(
        // sample.omega + autoPIDangular.calculate(pose.getRotation().getRadians(),
        // sample.heading)));
        // }

}
