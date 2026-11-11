package org.firstinspires.ftc.teamcode.game;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="NectarGateTuner")
public class NectarGateTuner extends LinearOpMode {
    @Override
    public void runOpMode() {
        Servo leftNectarGateServo = hardwareMap.get(Servo.class, "leftNectarGateServo");
        Servo rightNectarGateServo = hardwareMap.get(Servo.class, "rightNectarGateServo");

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftDrive");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftDrive");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightDrive");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightDrive");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        rightNectarGateServo.setDirection(Servo.Direction.REVERSE);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        boolean aPressed = false;
        boolean bPressed = false;

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a && (!aPressed)) {
                leftNectarGateServo.setPosition(leftNectarGateServo.getPosition() + 0.01);
                rightNectarGateServo.setPosition(rightNectarGateServo.getPosition() + 0.01);
                aPressed = true;
            }
            if (!gamepad1.a) {
                aPressed = false;
            }
            if (gamepad1.b && (!bPressed)) {
                leftNectarGateServo.setPosition(leftNectarGateServo.getPosition() - 0.01);
                rightNectarGateServo.setPosition(rightNectarGateServo.getPosition() - 0.01);
                bPressed = true;
            }
            if (!gamepad1.b) {
                bPressed = false;
            }

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            telemetry.addData("leftNectarGateServoPos", leftNectarGateServo.getPosition());
            telemetry.addData("rightNectarGateServoPos", rightNectarGateServo.getPosition());

            telemetry.update();
        }
    }
}
