// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final int MOTOR_ID = 12;
    public static final int JOYSTICK_ID = 5;
    public static final double TURRET_X = -1;
    public static final double TURRET_Y = -1;
    public static final double TARGET_X = -1;
    public static final double TARGET_Y = -1;
    public static final double OUR_ALLIACNE_X = -1;
    public static final double OUR_ALLIANCE_Y = -1;
    public static final int PULSE_PER_DEGREE = 261260 / 360;
    public static final int SENSOR_CLOCKWISE = 0;
    public static final int SENSOR_COUNTER_CLOCKWISE = 0;
    public static final int MOTION_RANGE = 190;
    public static final double SENSOR_ACIVATE_VALUE = -1;
    public static final double MIN_POWER_FOR_MOVEMENT = 0;
    public static double FF_VALUE = 0.039099;
    public static final double TURRET_MOTOR_KP = 0.03;
}
