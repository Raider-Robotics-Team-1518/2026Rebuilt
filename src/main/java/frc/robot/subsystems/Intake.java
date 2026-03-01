// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Intake extends SubsystemBase {
  final SparkMax intakeMotor = new SparkMax(Constants.Motors.intakeMotorID, MotorType.kBrushless);
  final SparkMax winchMotorLeft = new SparkMax(Constants.Motors.winchLeftMotorID, MotorType.kBrushless);
  final SparkMax winchMotorRight = new SparkMax(Constants.Motors.winchRightMotorID, MotorType.kBrushless);
  private final RelativeEncoder leftWinchEncoder;
  private final RelativeEncoder rightWinchEncoder;

  SparkBaseConfig intakeMotorConfig;
  SparkBaseConfig winchMotorLeftConfig;
  SparkBaseConfig winchMotorRightConfig;

  /** Creates a new Intake. */
  public Intake() {
    intakeMotorConfig = new SparkMaxConfig();
    intakeMotorConfig.idleMode(IdleMode.kCoast);
    intakeMotorConfig.inverted(false);
    intakeMotor.configure(intakeMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    leftWinchEncoder = winchMotorLeft.getEncoder();
    leftWinchEncoder.setPosition(0);
    rightWinchEncoder = winchMotorRight.getEncoder();
    rightWinchEncoder.setPosition(0);

    winchMotorLeftConfig.idleMode(IdleMode.kBrake);
    winchMotorLeft.configure(winchMotorLeftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    winchMotorRightConfig.idleMode(IdleMode.kBrake);
    winchMotorRightConfig.inverted(true);
    winchMotorRightConfig.follow(winchMotorLeft);
    winchMotorRight.configure(winchMotorRightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void runIntake() {
    intakeMotor.set(Constants.Speeds.intakeMotorSpeed);
  }

  public void stopIntake() {
    intakeMotor.stopMotor();
  }

  public void releaseWinchMotors() {
    winchMotorLeftConfig.idleMode(IdleMode.kCoast);
    winchMotorLeft.configureAsync(winchMotorLeftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    winchMotorRightConfig.idleMode(IdleMode.kCoast);
    winchMotorRightConfig.inverted(true);
    winchMotorRightConfig.follow(winchMotorLeft);
    winchMotorRight.configureAsync(winchMotorRightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void brakeWinchMotors() {
    winchMotorLeftConfig.idleMode(IdleMode.kBrake);
    winchMotorLeft.configureAsync(winchMotorLeftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    winchMotorRightConfig.idleMode(IdleMode.kBrake);
    winchMotorRightConfig.inverted(true);
    winchMotorRightConfig.follow(winchMotorLeft);
    winchMotorRight.configureAsync(winchMotorRightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void retractIntake() {
    // because the right winch motor is a follower, we don't need to set its speed here
    if (leftWinchEncoder.getPosition() > 10) {
      winchMotorLeft.set(Constants.Speeds.winchMotorIntakeSpeed);
    }  else {
      winchMotorLeft.stopMotor();
    }
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
