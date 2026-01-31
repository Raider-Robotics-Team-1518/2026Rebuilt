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

    private final AutoFactory factory;
 
    public Autos(Robot robot) {
        drivetrain = robot.drivetrain;
        robotContainer = robot.m_robotContainer;
        routines = robot.routines;
        autoPIDx = new PIDController(kAutoKp, kAutoKi, kAutoKd);
        autoPIDy = new PIDController(kAutoKp, kAutoKi, kAutoKd);
        autoPIDangular = new PIDController(kAutoAngularKp, kAutoAngularKi, kAutoAngularKd);
        autoPIDangular.enableContinuousInput(-Math.PI, Math.PI);

        // Create the auto factory - Need to look at how this needs to be implemented connecting CTRE with Choreo
        factory = new AutoFactory(this::getPose, drivetrain::resetPose, drivetrain::setControl,
                true, drivetrain);

    }       

    public Command example() {
        AutoRoutine routine = factory.newRoutine("LeaveHome");
        AutoTrajectory exampleTraj = routine.trajectory("LeaveHome");
        routine.active().onTrue(Commands.sequence(Commands.print("running example auto"), exampleTraj.resetOdometry(), Commands.print("Odometry Reset"), exampleTraj.cmd(), Commands.print("Trajectory Following")));
        exampleTraj.done().onTrue(Commands.sequence(Commands.print("Auto Completed"), routines.stopBot()));

        return routine.cmd();
    }

    private Pose2d getPose() {
        Optional<Pose2d> current_pose = drivetrain.samplePoseAt(Instant.now().getEpochSecond());
        if (!current_pose.isEmpty()) {
            return current_pose.get();
        }
        return null;
    }

    /**
     * Follows a Choreo trajectory by moving towards the next sample. This method
     * is not intended for use outside of creating an {@link AutoFactory}.
     *
     * @param sample The next trajectory sample.
     */
    public void followTrajectory(SwerveSample sample) {
        Pose2d pose = getPose();
        Commands.print("vx" + sample.vx);
        Commands.print("vy" + sample.vy);
        Commands.print("x" + sample.x);
        Commands.print("y" + sample.y);

        robotContainer.drive
                .withVelocityX(sample.vx + autoPIDx.calculate(pose.getX(), sample.x))
                .withVelocityY(sample.vy + autoPIDy.calculate(pose.getY(), sample.y))
                .withRotationalRate(
                        sample.omega + autoPIDangular.calculate(pose.getRotation().getRadians(), sample.heading));

    }
    public void driveRobotCentric(SwerveSample sample) {
        Pose2d pose = getPose();
        Commands.print("vx" + sample.vx);
        Commands.print("vy" + sample.vy);
        Commands.print("x" + sample.x);
        Commands.print("y" + sample.y);
            drivetrain.applyRequest(() -> 
                robotContainer.rcdrive
                .withVelocityX(sample.vx + autoPIDx.calculate(pose.getX(), sample.x))
                .withVelocityY(sample.vy + autoPIDy.calculate(pose.getY(), sample.y))
                .withRotationalRate(
                        sample.omega + autoPIDangular.calculate(pose.getRotation().getRadians(), sample.heading)));
    }

}
