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
  //private TalonFX shootMotor;
  private double shoot_velocity = 0;

  final SparkMax shootMotor = new SparkMax(Constants.Motors.shootNeoVortex, MotorType.kBrushless);
  SparkBaseConfig shootMotorConfig;

  private final double kPshoot = 0.002;
  private final double kIshoot = 0;
  private final double kDshoot = 0;

  SparkClosedLoopController m_controller;

  // kick motor
  private double kick_velocity = 0;

  final SparkMax kickerMotor = new SparkMax(Constants.Motors.kickerMotorID, MotorType.kBrushless);
  SparkBaseConfig kickerMotorConfig;

  private final double kPkick = 0.002;
  private final double kIkick = 0;
  private final double kDkick = 0;

  SparkClosedLoopController m_kicker_controller;

  public Shoot() {
    //shootMotor = new TalonFX(Constants.Motors.shootTalonFX);
    //shootMotor.setNeutralMode(NeutralModeValue.Coast);
    shootMotorConfig = new SparkMaxConfig();
    shootMotorConfig.idleMode(IdleMode.kCoast);
    shootMotorConfig.inverted(false);
    m_controller = shootMotor.getClosedLoopController();

    shootMotorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(kPshoot, kIshoot, kDshoot, ClosedLoopSlot.kSlot0);

    shootMotor.configure(shootMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    kickerMotorConfig = new SparkMaxConfig();
    kickerMotorConfig.idleMode(IdleMode.kBrake);
    kickerMotorConfig.inverted(false);
    m_kicker_controller = kickerMotor.getClosedLoopController();

    kickerMotorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(kPkick, kIkick, kDkick, ClosedLoopSlot.kSlot0);

    kickerMotor.configure(kickerMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void setShooterSpeed(int shooterVelocity) {
    shoot_velocity = shooterVelocity;
    //VelocityVoltage velocityControl = new VelocityVoltage(velocity).withSlot(0);
    //m_controller.setSetpoint(shooterVelocity, ControlType.kVelocity);
    shootMotor.set(shooterVelocity/10);
  }

  public void setKickerSpeed(int kickerVelocity) {
    kick_velocity = kickerVelocity;
    //VelocityVoltage velocityControl = new VelocityVoltage(velocity).withSlot(0);
    //m_kicker_controller.setSetpoint (kickerVelocity, ControlType.kVelocity);
    kickerMotor.set(kickerVelocity/10);
  }

  public void stopShooter() {
    shootMotor.stopMotor();
  }

  public void stopKicker() {
    kickerMotor.stopMotor();
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ShooterRPM", shoot_velocity);
    SmartDashboard.putNumber("KickerRPM", kick_velocity);
  }
}
