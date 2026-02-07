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
        DEFAULT, HOME, TRACKING, SEARCHING
    }

    public static final class Motors {
        public static final int intakeNeoVortex = 24;
        public static final int shootTalonFX = 25;
        public static final int turretMotorID = 26;
        public static final int turretEncoderID = 27;
    }

    public static final class Speeds {
        public static final double intakeMotorSpeed = 0.5;
        public static final double shootMotorSpeed = 85;  //Initial testing 85 to 100 is fairly consistent
    }

    public static final class Dimensions {
        public static final Distance limelightHeight = Distance.ofRelativeUnits(5, Inches);
        public static final Angle limelightMountingAngle = Angle.ofRelativeUnits(0, Degrees);
        public static final Distance limelightXOffset = Distance.ofRelativeUnits(0, Inches);
        public static final Distance limelightYOffset = Distance.ofRelativeUnits(5, Inches);
        public static final Distance targetHeight = Distance.ofRelativeUnits(72, Inches);
    }
}
