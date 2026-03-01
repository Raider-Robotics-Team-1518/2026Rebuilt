// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootFuel extends Command {
  /** Creates a new ShootFuel. */

  private boolean isDone = false;
  private int shootSpeed = 0;
  private static final InterpolatingDoubleTreeMap kRegression = new InterpolatingDoubleTreeMap();

  public ShootFuel(int shootSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.shootSystem, RobotContainer.turretControl);
    this.shootSpeed = shootSpeed;
    //this.kickSpeed = kickSpeed;
    {
        // first value is distance to target in INCHES
        // second parameter is the corresponding shooter RPMs
        // we'll want a few known values to give the interpolator

        // closest we can shoot
        kRegression.put(36.0, 2000.0);
        kRegression.put(72.0, 2500.0);
        kRegression.put(120.0, 2500.0);
        kRegression.put(150.0, 2500.0);
        kRegression.put(180.0, 3000.0);
        // farthest we can shoot
        kRegression.put(240.0, 4000.0);
    }

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Distance distAsDistance = RobotContainer.turretControl.getDistance();
    double distance = distAsDistance.in(Units.Inches);
    double shooterSpeed = 0;
    if (distance > 0) {
      // if we get a distance value, it means we're seeing the target AprilTag
      // so use the calculated distance to the hub
      shooterSpeed = kRegression.get(distance);
    } else {
      // otherwise, use the shoot speed passed in since we're probably in the
      // neutral zone shooting towards our alliance side
      shooterSpeed = this.shootSpeed;
    }
    if (Math.abs(shooterSpeed) > 0) {
      RobotContainer.shootSystem.setShooterSpeed((int) shooterSpeed);
      //RobotContainer.shootSystem.setKickerSpeed(kickSpeed);
    } else {
      RobotContainer.shootSystem.stopShooter();
      //RobotContainer.shootSystem.stopKicker();
    }
    isDone = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // RobotContainer.shootSystem.stopShooter();
    //RobotContainer.shootSystem.stopKicker();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
