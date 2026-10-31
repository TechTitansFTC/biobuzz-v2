package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class Constants {
    // Drive
    public static RevHubOrientationOnRobot.LogoFacingDirection CONTROL_HUB_LOGO_FACING_DIRECTION = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
    public static RevHubOrientationOnRobot.UsbFacingDirection CONTROL_HUB_USB_FACING_DIRECTION = RevHubOrientationOnRobot.UsbFacingDirection.DOWN;
    public static String TEAM = "R";

    // Intake
    public static double INTAKE_MOTOR_POWER = 0.75; // Motor Power
    public static double REVERSE_INTAKE_MOTOR_POWER = 0.3; // Magnitude
    public static double OUT_TAKE_TIME = 0.35; // Seconds

    // Storage
    public static double COLOR_SENSOR_PROXIMITY = 3; // Centimeters, how far the color sensor should look to see if nectar is there or not
    public static double COLOR_SENSOR_COLOR_THRESHOLD = 20; // Degrees on the HSV scale
    public static double DISTANCE_SENSOR_THRESHOLD = 3; // Centimeters
    public static double TRANSFER_MOTOR_POWER = 0.63; // Motor power
    public static double REVERSE_TRANSFER_MOTOR_POWER = 0.3; // Motor power magnitude

    // Scoring
    public static double ENCODER_TO_TURRET_RATIO = 1.16 / 3.2;
    public static double TURRET_TO_SERVO_RATIO = 1 / 1.16;
    public static double[] POLLEN_TURRET_SERVO_CONTROLLER = new double[]{0.06, 0.0001, 0.005}; // p, i, d
    public static double[] NECTAR_TURRET_SERVO_CONTROLLER = new double[]{0.06, 0.0001, 0.005};
    public static double[] POLLEN_TURRET_MOTOR_CONTROLLER = new double[]{0.5, 0.0001, 0.005};
    public static double[] NECTAR_TURRET_MOTOR_CONTROLLER = new double[]{0.5, 0.0001, 0.005};
}
