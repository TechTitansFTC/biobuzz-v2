package org.firstinspires.ftc.teamcode.util;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

@Configurable
public class Constants {
    // Drive
    public static RevHubOrientationOnRobot.LogoFacingDirection CONTROL_HUB_LOGO_FACING_DIRECTION = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
    public static RevHubOrientationOnRobot.UsbFacingDirection CONTROL_HUB_USB_FACING_DIRECTION = RevHubOrientationOnRobot.UsbFacingDirection.DOWN;
    public static String TEAM = "R";

    // Intake
    public static double INTAKE_MOTOR_POWER = 1;
    public static double REVERSE_INTAKE_MOTOR_POWER = 0.5;
    public static double OUT_TAKE_TIME = 0.2;
    public static double FLOWER_RIGHT_SERVO_DOWN = 0.8022;
    public static double FLOWER_RIGHT_SERVO_UP = 0.4317;
    public static double FLOWER_LEFT_SERVO_DOWN = 0.6094;
    public static double FLOWER_LEFT_SERVO_UP = 0.2394;

    // Storage
    public static double POLLEN_GATE_OPEN_POSITION = 0.6;
    public static double POLLEN_GATE_CLOSE_POSITION = 0.1;
    public static double LEFT_NECTAR_GATE_OPEN_POSITION = 0.6;
    public static double LEFT_NECTAR_GATE_CLOSE_POSITION = 0.6;
    public static double RIGHT_NECTAR_GATE_OPEN_POSITION = 0.6;
    public static double RIGHT_NECTAR_GATE_CLOSE_POSITION = 0.6;
    public static double NECTAR_GATE_HOLD_NECTAR_POSITION = 0.6;
    public static double COLOR_SENSOR_PROXIMITY = 2.75;
    public static double COLOR_SENSOR_COLOR_THRESHOLD = 20.0;
    public static double DISTANCE_SENSOR_THRESHOLD = 3.0;
    public static double TRANSFER_MOTOR_POWER = 1;
    public static double REVERSE_TRANSFER_MOTOR_POWER = 0.5;
        // Color
        public static double YELLOW_LOWER_BOUND = 40;
        public static double YELLOW_UPPER_BOUND = 150;
        public static double BLUE_LOWER_BOUND = 200;
        public static double BLUE_UPPER_BOUND = 270;
        public static double RED_LOWER_BOUND = 300;
        public static double RED_UPPER_BOUND = 40;

    // Scoring General
    public static double ENCODER_TO_TURRET_RATIO = 1.0 / 2.75;
    public static double TURRET_OFFSET_DISTANCE = 5.669;
    public static double POLLEN_VELOCITY_MULTIPLIER = 0.06;
    public static double NECTAR_VELOCITY_MULTIPLIER = 0.07;
    public static double ALIGN_IMU_TIME = 0.6;
    public static double ALIGN_APRIL_TAG_TIME = 0.5;
    public static double SHOOTING_TIME = 0.4;

    public static double POLLEN_SERVO_KP = 0.003, POLLEN_SERVO_KI = 0, POLLEN_SERVO_KD = 0.000000004, POLLEN_SERVO_KF = 0.06;
  // Pollen shooter velocity control (values from ShootingTest)
    // power = (kS + kV * targetRPM) * (NOMINAL_VOLTAGE / battery) + PID(targetRPM - rpm)
    public static double POLLEN_SHOOTER_TARGET_RPM = 3200;
    public static double POLLEN_SHOOTER_RPM_TOLERANCE = 50;      // shoot once within +/- this
    public static double POLLEN_SHOOTER_KS = 0.0;
    public static double POLLEN_SHOOTER_KV = 0.00017;
    public static double POLLEN_SHOOTER_KP = 0.0007;
    public static double POLLEN_SHOOTER_KI = 0.0;
    public static double POLLEN_SHOOTER_KD = 0.0;
    public static double POLLEN_SHOOTER_RPM_FILTER = 0.3;        // 1 = no smoothing, lower = smoother
    public static boolean POLLEN_SHOOTER_MOTOR_REVERSED = true;     // flips which way the wheel spins
    public static boolean POLLEN_SHOOTER_ENCODER_REVERSED = true;   // flips only the RPM reading
    public static double NOMINAL_VOLTAGE = 12.0;
    public static double POLLEN_ALIGN_TIMEOUT = 3.0;             // give up if not up to speed by then

    // Pollen turret AprilTag alignment (copy the kp/ki/kd/kf/reverse you tuned in CameraTest)
    public static double POLLEN_TURRET_TAG_KP = 0.003;
    public static double POLLEN_TURRET_TAG_KI = 0.0;
    public static double POLLEN_TURRET_TAG_KD = 0.000000004;
    public static double POLLEN_TURRET_TAG_KF = 0.06;
    public static boolean POLLEN_TURRET_TAG_REVERSED = true;
    public static double NECTAR_SERVO_KP = 0.06, NECTAR_SERVO_KI = 0.0001, NECTAR_SERVO_KD = 0.005, NECTAR_SERVO_KF = 0.0;
    public static double POLLEN_MOTOR_KP = 0.5, POLLEN_MOTOR_KI = 0.0001, POLLEN_MOTOR_KD = 0.005, POLLEN_MOTOR_KF = 0.005;
    public static double NECTAR_MOTOR_KP = 0.5, NECTAR_MOTOR_KI = 0.0001, NECTAR_MOTOR_KD = 0.005, NECTAR_MOTOR_KF = 0.005;

    public static double[] POLLEN_TURRET_SERVO_CONTROLLER = new double[]{POLLEN_SERVO_KP, POLLEN_SERVO_KI, POLLEN_SERVO_KD, POLLEN_SERVO_KF};
    public static double[] NECTAR_TURRET_SERVO_CONTROLLER = new double[]{NECTAR_SERVO_KP, NECTAR_SERVO_KI, NECTAR_SERVO_KD, NECTAR_SERVO_KF};
    public static double[] POLLEN_TURRET_MOTOR_CONTROLLER = new double[]{POLLEN_MOTOR_KP, POLLEN_MOTOR_KI, POLLEN_MOTOR_KD, POLLEN_MOTOR_KF};
    public static double[] NECTAR_TURRET_MOTOR_CONTROLLER = new double[]{NECTAR_MOTOR_KP, NECTAR_MOTOR_KI, NECTAR_MOTOR_KD, NECTAR_MOTOR_KF};

    public static double POLLEN_TURRET_VELOCITY_MODEL(double distanceInches) { return distanceInches * POLLEN_VELOCITY_MULTIPLIER; }
    public static double NECTAR_TURRET_VELOCITY_MODEL(double distanceInches) { return distanceInches * NECTAR_VELOCITY_MULTIPLIER; }

    // Autonomous
    public static double PARK_TIME = 5.0;
    public static double SHOOTER_SPIN_UP_TIME = 3.0;
    public static double AUTO_INTAKE_TIME = 1.0;
    public static double APRIL_TAG_ALIGN_DETECTION_TIME = 1.0;
}