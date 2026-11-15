package org.firstinspires.ftc.teamcode.game.tests.isolatedTests;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Nectar Gate Tuner")
public class NectarGateTuner extends LinearOpMode {
    @Override
    public void runOpMode() {
        Servo leftNectarGateServo = hardwareMap.get(Servo.class, "leftNectarGate");
        Servo rightNectarGateServo = hardwareMap.get(Servo.class, "rightNectarGate");
        Servo pollenGateServo = hardwareMap.get(Servo.class, "pollenGate");

//        rightNectarGateServo.setDirection(Servo.Direction.REVERSE);

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.aWasPressed()) {
                leftNectarGateServo.setPosition(leftNectarGateServo.getPosition() - 0.01);
            }
            if (gamepad1.bWasPressed()) {
                leftNectarGateServo.setPosition(leftNectarGateServo.getPosition() + 0.01);
            }
            if (gamepad1.yWasPressed()) {
                rightNectarGateServo.setPosition(rightNectarGateServo.getPosition() + 0.01);
            }
            if (gamepad1.xWasPressed()) {
                rightNectarGateServo.setPosition(rightNectarGateServo.getPosition() - 0.01);
            }
            if (gamepad1.leftBumperWasPressed()) {
                pollenGateServo.setPosition(pollenGateServo.getPosition() - 0.01);
            }
            if (gamepad1.rightBumperWasPressed()) {
                pollenGateServo.setPosition(pollenGateServo.getPosition() + 0.01);
            }

            telemetry.addData("leftNectarGateServoPos", leftNectarGateServo.getPosition());
            telemetry.addData("rightNectarGateServoPos", rightNectarGateServo.getPosition());
            telemetry.addData("pollenGateServoPos", pollenGateServo.getPosition());

            telemetry.update();
        }
    }
}
