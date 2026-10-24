package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.seattlesolvers.solverslib.command.SubsystemBase;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumDriveSubsystem extends SubsystemBase {
    private final IMU imu;
    private final DcMotor frontLeft, frontRight, backLeft, backRight;

    private final Telemetry telemetry;

    public MecanumDriveSubsystem(HardwareMap hardwareMap, IMU imu, Telemetry telemetry) {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        this.imu = imu;
        this.telemetry = telemetry;

        register();
    }

    @Override
    public void periodic() {
        telemetry.addData("backLeftMotorPower", getBackLeftMotorPower());
        telemetry.addData("backRightMotorPower", getBackRightMotorPower());
        telemetry.addData("frontLeftMotorPower", getFrontLeftMotorPower());
        telemetry.addData("frontRightMotorPower", getFrontRightMotorPower());
    }

    public void fieldCentricDrive(double x, double y, double rotation) {
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rotation), 1);
        double frontLeftPower = (rotY + rotX + rotation) / denominator;
        double backLeftPower = (rotY - rotX + rotation) / denominator;
        double frontRightPower = (rotY - rotX - rotation) / denominator;
        double backRightPower = (rotY + rotX - rotation) / denominator;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }

    public void robotCentricDrive(double x, double y, double rotation) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rotation), 1);
        double frontLeftPower = (y + x + rotation) / denominator;
        double backLeftPower = (y - x + rotation) / denominator;
        double frontRightPower = (y - x - rotation) / denominator;
        double backRightPower = (y + x - rotation) / denominator;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }

    public void resetIMU() {
        imu.resetYaw();
    }

    public double getBackLeftMotorPower() { return backLeft.getPower(); }
    public double getBackRightMotorPower() { return backRight.getPower(); }
    public double getFrontLeftMotorPower() { return frontLeft.getPower(); }
    public double getFrontRightMotorPower() { return frontRight.getPower(); }

}
