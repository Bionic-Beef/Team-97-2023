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
    // PID constants
    public static double GyroYKP = 0.030;
    public static double GyroYKI = 0.001;
    public static double GyroYKD = 0;

    public static double GyroZKP = 0.004;
    public static double GyroZKI = 0;
    public static double GyroZKD = 0;

    public static double EncoderPIDKP = 0.25;
    public static double EncoderPIDKI = 0.05;
    public static double EncoderPIDKD = 0;

    //encoder revolutions to wheel revolutions
    public static double driveTrainGearRatio = 7;

    // Auto command constants
    public static double timeToRotateChuteInAuto = 1;

    //distances are inverse of what we consider the "front" of the robot
    public static double distanceToLeaveCommunityFromStart = 75;
    public static double distanceToChargeStationFromOutsideCommunity = -140;
    public static double distanceToChargeStationFromStart = 60;
    public static double wheelRadius = 3.5;
    public static double chuteSpeed  = .35;
    public static double chuteSpeedBackward = 1;
    public static double autoConstantSpeed = .70;
}

