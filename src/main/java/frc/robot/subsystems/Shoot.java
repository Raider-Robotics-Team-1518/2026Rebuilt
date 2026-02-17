// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shoot extends SubsystemBase {
  /** Creates a new Shoot. */
  private double shoot_velocity = 0;
  private final SparkMax shootMotor = new SparkMax(Constants.Motors.shootMotorID, MotorType.kBrushless);
  private final SparkClosedLoopController m_shoot_controller;
  private final SparkMaxConfig shootMotorConfig;
  private final RelativeEncoder shootEncoder;
  private final double kPshoot = 0.00002;
  private final double kIshoot = 0;
  private final double kDshoot = 0.001;
  private final double kFshoot = 7.55;


  // kick motor
  private double kick_velocity = 0;
  private final SparkMax kickerMotor;
  private final SparkMaxConfig kickerMotorConfig;
  private final SparkClosedLoopController m_kicker_controller;
  private final double kPkick = 0.000015;
  private final double kIkick = 0;
  private final double kDkick = 0.005;
  private final double kFkick = 0.0025;


  public Shoot() {
    kickerMotor = new SparkMax(Constants.Motors.kickerMotorID, MotorType.kBrushless);
    m_shoot_controller = shootMotor.getClosedLoopController();
    shootEncoder = shootMotor.getEncoder();
    shootMotorConfig = new SparkMaxConfig();
    shootMotorConfig.idleMode(IdleMode.kCoast);
    shootMotorConfig.inverted(false);
    shootMotorConfig.encoder
      .positionConversionFactor(1)
      .velocityConversionFactor(1);
    shootMotorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(kPshoot, kIshoot, kDshoot, ClosedLoopSlot.kSlot0)
      .feedForward.kS(kFshoot);
    shootMotor.configure(shootMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    kickerMotorConfig = new SparkMaxConfig();
    kickerMotorConfig.idleMode(IdleMode.kBrake);
    kickerMotorConfig.inverted(true);
    kickerMotorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(kPkick, kIkick, kDkick, ClosedLoopSlot.kSlot0)
      .feedForward.kS(kFkick);
    kickerMotor.configure(kickerMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    m_kicker_controller = kickerMotor.getClosedLoopController();

  }

  public void setShooterSpeed(int shooterVelocity) {
    shoot_velocity = shooterVelocity;
    m_shoot_controller.setSetpoint(4000, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    // shootMotor.set(shoot_velocity/100);
  }

  public void setKickerSpeed(int kickerVelocity) {
    kick_velocity = kickerVelocity;
    m_kicker_controller.setSetpoint(2500, ControlType.kVelocity);
    // kickerMotor.set(kick_velocity/100);
  }

  public void stopShooter() {
    // m_shoot_controller.setSetpoint(0, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    shootMotor.stopMotor();
  }

  public void stopKicker() {
    // m_kicker_controller.setSetpoint(0, ControlType.kVelocity);
    kickerMotor.stopMotor();
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ShooterRPM", shootMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("KickerRPM", kickerMotor.getEncoder().getVelocity());
  }
}
