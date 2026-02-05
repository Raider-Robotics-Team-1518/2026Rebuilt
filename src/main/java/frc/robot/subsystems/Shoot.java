// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

public class Shoot extends SubsystemBase {
  /** Creates a new Shoot. */
  private TalonFX shootMotor;
  private double velocity = 0;

  public Shoot() {
    shootMotor = new TalonFX(Constants.Motors.shootTalonFX);
    shootMotor.setNeutralMode(NeutralModeValue.Coast);

    var slot0Configs = new Slot0Configs();
      slot0Configs.kP = 0.2;
      slot0Configs.kI = 0.0;
      slot0Configs.kD = 0.01;
      shootMotor.getConfigurator().apply(slot0Configs);
  }

  public void setShooterSpeed(double shooterVelocity) {
    velocity = shooterVelocity;
    VelocityVoltage velocityControl = new VelocityVoltage(velocity).withSlot(0);
    shootMotor.setControl(velocityControl.withVelocity(velocity));
  }

  public void stopShooter() {
    shootMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ShooterRPM", velocity);
  }
}
