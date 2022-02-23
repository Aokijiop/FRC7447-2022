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
    // 6 motor controllers
    // 1 Spark motor controller for arm

    // Device numbers for the motors
    public static final int leftFront = 0;
    public static final int rightFront = 2;
    public static final int leftBack = 1;
    public static final int rightBack = 3;
    public static final int dumperPort = 7;
    public static final int dumperArmPort = 8;

    // DIO Ports
    public static final int leftEncoderA = 0;
    public static final int leftEncoderB = 1;
    public static final int rightEncoderA = 2;
    public static final int rightEncoderB = 3;

    // Time Values for Time-Based Commands
    public static final double drive_fwd_time = 3.0;

    // Speed Constants
    public static final double mSpeed = 0.6;
    public static final double tSpeed = 0.6;
    public static final double speedBoost = 0.85;
    public static final double autonSpeed = 0.4;

    // Dumper Constants
    public static final double dumperPower = 0.5;
    public static final double dumperRetch = -0.5;
    public static final double dumperUpSpeed =  0.5;
    public static final double dumperDownSpeed = -0.5;
    
    // Joystick port and axis numbers
    public static final int joystickPort = 0;
    public static final int x_axis = 0;
    public static final int y_axis = 1;
    public static final int z_axis = 2;
    public static final int z_rotate = 3;
    public static final int xButton = 1;
    public static final int aButton = 2;
    public static final int bButton = 3;
    public static final int yButton = 4;
    public static final int LButton = 5;
    public static final int RButton = 6;
    public static final int LTrigger = 7;
    public static final int RTrigger = 8;

    // Turn To Angle Velocity Setpoints
    public static final double TTALeftVelocity = 0.0;
    public static final double TTARightVelocity = 0.0;

    // Drie to Distance Velocity Setpoints
    public static final double DTDLeftVelocity = 0.0;
    public static final double DTDRightVelocity = 0.0;
}
