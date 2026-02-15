// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
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
  private final SparkBaseConfig shootMotorConfig;
  private final SparkClosedLoopController m_shoot_controller;
  private final double kPshoot = 0.002;
  private final double kIshoot = 0;
  private final double kDshoot = 0;
  private final double kFshoot = 0;


  // kick motor
  private double kick_velocity = 0;
  private final SparkMax kickerMotor = new SparkMax(Constants.Motors.kickerMotorID, MotorType.kBrushless);
  private final SparkBaseConfig kickerMotorConfig;
  private final SparkClosedLoopController m_kicker_controller;
  private final double kPkick = 0.002;
  private final double kIkick = 0;
  private final double kDkick = 0;
  private final double kFkick = 0;


  public Shoot() {
    shootMotorConfig = new SparkMaxConfig();
    shootMotorConfig.idleMode(IdleMode.kCoast);
    shootMotorConfig.inverted(false);
    shootMotorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(kPshoot, kIshoot, kDshoot, ClosedLoopSlot.kSlot0);
    shootMotor.configure(shootMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    m_shoot_controller = shootMotor.getClosedLoopController();

    kickerMotorConfig = new SparkMaxConfig();
    kickerMotorConfig.idleMode(IdleMode.kBrake);
    kickerMotorConfig.inverted(true);
    kickerMotorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(kPkick, kIkick, kDkick, ClosedLoopSlot.kSlot0);
    kickerMotor.configure(kickerMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    m_kicker_controller = kickerMotor.getClosedLoopController();
  }

  public void setShooterSpeed(int shooterVelocity) {
    shoot_velocity = shooterVelocity;
    m_shoot_controller.setSetpoint(shoot_velocity, ControlType.kVelocity);
    //shootMotor.set(shoot_velocity);
  }

  public void setKickerSpeed(int kickerVelocity) {
    kick_velocity = kickerVelocity;
    m_kicker_controller.setSetpoint(kick_velocity, ControlType.kVelocity);
    //kickerMotor.set(kickerVelocity/10);
  }

  public void stopShooter() {
    m_shoot_controller.setSetpoint(0, ControlType.kVelocity);
  }

  public void stopKicker() {
    m_kicker_controller.setSetpoint(0, ControlType.kVelocity);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ShooterRPM", shoot_velocity);
    SmartDashboard.putNumber("KickerRPM", kick_velocity);
  }
}
