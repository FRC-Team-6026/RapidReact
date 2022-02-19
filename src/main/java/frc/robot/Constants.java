// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double ksVolts = 0.15;
    public static final double kvVoltSecondsPerMeter = 2.92;
    public static final double kvVoltMinutesPerMotorRotation = 0.00218;
    public static final double kaVoltSecondsSquaredPerMeter = 0.335;
    public static final double kaVoltMinuteSecondsPerMotorRotation = 0.000249;
    public static final double kPDriveVel = 2.18;

    public static final double kTrackWidthMeters = .55625;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackWidthMeters);

    public static final double kMaxSpeedMetersPerSecond = 1;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;

    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    public static final double kGearRatio = 10.71;
    public static final double kMetersPerWheelRevolution = 0.478778316;
    public static final double kMetersPerMotorRevolution = kMetersPerWheelRevolution / kGearRatio;
    public static final double kMetersPerSecondPerRPM = kMetersPerMotorRevolution / 60.0;

    //Spark max PID constants
    public static final double kMaxRpm = 4500;
    public static final double kMaxAccelRpmPerSec = 1500;
    public static final double kRotationDiffRpm = 1500;
    public static final double kFf = 0;
    public static final double kP = 10e-10;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double minOutput = -1;
    public static final double maxOutput = 1;
}
