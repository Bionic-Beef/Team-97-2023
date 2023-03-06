// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static double GyroYKP = 0.04;
    public static double GyroYKI = 0;
    public static double GyroYKD = 0.1;

    public static double GyroZKP = 0.004;
    public static double GyroZKI = 0;
    public static double GyroZKD = 0.1;

    public static double EncoderPIDKP = 0.25;
    public static double EncoderPIDKI = 0.05;
    public static double EncoderPIDKD = 0;

    public static double driveTrainGearRatio = 12.75;

    public static double timeToRotateChuteInAuto;
}
