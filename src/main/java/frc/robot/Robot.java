// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    public final RobotContainer m_robotContainer;
    public final CommandSwerveDrivetrain drivetrain;
    public final Routines routines;
    public final Autos autos;
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
            .withTimestampReplay()
            .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();
        drivetrain = m_robotContainer.drivetrain;
        // Initialize compositions
        routines = new Routines(this);
        autos = new Autos(this);

        // add options to the dashboard
        autoChooser.addOption("Drive8_Test", autos.Drive8_Test());
        autoChooser.addOption("LeaveHome", autos.example());
        autoChooser.addOption("Test_45", autos.Test_45Command());
        autoChooser.addOption("ScorePre_GoDepot", autos.ScorePre_GoDepotCommand());
        SmartDashboard.putData("Select Auto", autoChooser);
    }

    public Command getAuto() {
        return autoChooser.getSelected();
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        m_autonomousCommand = autoChooser.getSelected();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
