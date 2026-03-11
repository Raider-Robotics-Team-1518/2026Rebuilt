// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

/** Add your docs here. */
public final class Constants {
    public enum turretStates {
        DEFAULT, HOME, TRACKING, SEARCHING, MANUAL
    }

    public static final double turretMotorDegreesPerRotation = 900;

    public static final class Motors {
        public static final int intakeMotorID = 24;
        public static final int shootMotorID = 25;
        public static final int turretMotorID = 26;
        public static final int turretEncoderID = 27;
        public static final int kickerMotorID = 28;
        public static final int winchRightMotorID = 14;
        public static final int winchLeftMotorID = 29;
        public static final int indexMotorID = 30;
    }

    public static final class Speeds {
        public static final double intakeMotorSpeed = 1;
        public static final double winchMotorIntakeSpeed = 0.35;
        public static final double shootMotorSpeed = 0.55;
        public static final int kickMotorSpeed = 2700;
        public static final int backKickMotorSpeed = -2500;
        public static final double indexMotorSpeed = -0.5;
        public static final double neoRPM = 11710;
        public static final double turretMotorFactor = 0.0001;
    }

    public static final class Dimensions {
        public static final Distance limelightHeight = Distance.ofRelativeUnits(18, Inches);
        public static final Angle limelightMountingAngle = Angle.ofRelativeUnits(0, Degrees);
        public static final Distance limelightXOffset = Distance.ofRelativeUnits(0, Inches);
        public static final Distance limelightYOffset = Distance.ofRelativeUnits(0, Inches);
        public static final Distance targetHeight = Distance.ofRelativeUnits(44.25, Inches);
    }
}
