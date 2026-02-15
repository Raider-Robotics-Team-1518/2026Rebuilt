// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Intake extends SubsystemBase {
  final SparkMax intakeMotor = new SparkMax(Constants.Motors.intakeMotorID, MotorType.kBrushless);
  SparkBaseConfig intakeMotorConfig;
  /** Creates a new Intake. */
  public Intake() {
    intakeMotorConfig = new SparkMaxConfig();
    intakeMotorConfig.idleMode(IdleMode.kCoast);
    intakeMotorConfig.inverted(false);
    intakeMotor.configure(intakeMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

  }

  public void runIntake() {
    intakeMotor.set(Constants.Speeds.intakeMotorSpeed);
  }

  public void stopIntake() {
    intakeMotor.stopMotor();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
