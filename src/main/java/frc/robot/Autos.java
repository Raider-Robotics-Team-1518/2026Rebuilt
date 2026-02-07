package frc.robot;

import java.time.Instant;
import java.util.Optional;

import choreo.auto.AutoFactory;
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

    public Command example() {
        // create a new Routine with the name "LeaveHome" - which should match the
        // trajectory you'll create next
        AutoRoutine routine = autoFactory.newRoutine("LeaveHome");
        // Read in the trajectory named "LeaveHome" - this has to be the case-sensitive
        // name from Choreo
        AutoTrajectory exampleTraj = routine.trajectory("LeaveHome");
        // when the routine is active (auto is enabled) run a sequence of commands - print a message, reset odometry,
        // then drive our trajectory
        routine.active().onTrue(Commands.sequence(Commands.print("running example auto"), exampleTraj.resetOdometry(),
                exampleTraj.cmd()));
        // finally, when that trajectory is done, stop the bot using the routine from Routines.java
        exampleTraj.done().onTrue(Commands.sequence(Commands.print("Auto Completed"), routines.stopBot()));
        return routine.cmd();
    }

        public Command Drive8_Test() {
        // create a new Routine with the name "Drive8_Test" - which should match the
        // trajectory you'll create next
        AutoRoutine routine = autoFactory.newRoutine("Drive8_Test");
        // Read in the trajectory named "LeaveHome" - this has to be the case-sensitive
        // name from Choreo
        AutoTrajectory Drive8Traj = routine.trajectory("Drive8_Test");
        // when the routine is active (auto is enabled) run a sequence of commands - print a message, reset odometry,
        // then drive our trajectory
        routine.active().onTrue(Commands.sequence(Commands.print("running Drive8_Test auto"), Drive8Traj.resetOdometry(),
                Drive8Traj.cmd()));
        // finally, when that trajectory is done, stop the bot using the routine from Routines.java
        Drive8Traj.done().onTrue(Commands.sequence(Commands.print("Auto Completed"), routines.stopBot()));
        return routine.cmd();
    }
   
    public Command Test_45Command() {
        // create a new Routine with the name "Drive8_Test" - which should match the
        // trajectory you'll create next
        AutoRoutine routine = autoFactory.newRoutine("Test_45");
        // Read in the trajectory named "LeaveHome" - this has to be the case-sensitive
        // name from Choreo
        AutoTrajectory test_45Trajectory = routine.trajectory("Test_45");
        // when the routine is active (auto is enabled) run a sequence of commands - print a message, reset odometry,
        // then drive our trajectory
        routine.active().onTrue(Commands.sequence(Commands.print("running Test_45"), test_45Trajectory.resetOdometry(),
                test_45Trajectory.cmd()));
        // finally, when that trajectory is done, stop the bot using the routine from Routines.java
        test_45Trajectory.done().onTrue(Commands.sequence(Commands.print("Auto Completed"), routines.stopBot()));
        return routine.cmd();
    }
    public Command ScorePre_GoDepotCommand() {
        // create a new Routine with the name "Drive8_Test" - which should match the
        // trajectory you'll create next
        AutoRoutine routine = autoFactory.newRoutine("ScorePre_GoDepot");
        // Read in the trajectory named "LeaveHome" - this has to be the case-sensitive
        // name from Choreo
        AutoTrajectory ScorePre_GoDepotTrajectory = routine.trajectory("ScorePre_GoDepot");
        // when the routine is active (auto is enabled) run a sequence of commands - print a message, reset odometry,
        // then drive our trajectory
        routine.active().onTrue(Commands.sequence(Commands.print("running ScorePre_GoDepot"), ScorePre_GoDepotTrajectory.resetOdometry(),
                ScorePre_GoDepotTrajectory.cmd()));
        // finally, when that trajectory is done, stop the bot using the routine from Routines.java
        ScorePre_GoDepotTrajectory.done().onTrue(Commands.sequence(Commands.print("Auto Completed"), routines.stopBot()));
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
