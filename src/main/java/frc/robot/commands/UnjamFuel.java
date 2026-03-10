// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class UnjamFuel extends Command {
  /** Creates a new UnjamFuel. */
  //private int unjamSpeed = 0;
  private boolean isDone = false;

  public UnjamFuel() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.shootSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(Constants.Speeds.indexMotorSpeed) > 0) {
      RobotContainer.shootSystem.setUnjamSpeed();
    } else {
      RobotContainer.shootSystem.stopUnjam();
    }
    isDone = true;
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // RobotContainer.shootSystem.stopKicker();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
