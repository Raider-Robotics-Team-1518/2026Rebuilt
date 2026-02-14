package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ShootFuel;
import frc.robot.subsystems.CommandSwerveDrivetrain;


public class Routines {
    private final CommandSwerveDrivetrain drivetrain;
    private final RobotContainer robotContainer;
    
    public Routines(Robot robot) {
        drivetrain = robot.drivetrain;
        robotContainer = robot.m_robotContainer;
    }
    
    /**
     * A routine to stop the bot.
     */
    public Command stopBot() {
        return Commands.sequence( drivetrain.applyRequest(() ->
                robotContainer.drive.withVelocityX(0)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            ));
    }

    public Command timedShoot() {
        return Commands.sequence(new ShootFuel(Constants.Speeds.shootMotorSpeed, 0).withTimeout(5),
        new ShootFuel(0, 0));
    }
}
