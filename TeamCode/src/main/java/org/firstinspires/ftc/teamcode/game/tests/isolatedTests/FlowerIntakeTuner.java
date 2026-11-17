package org.firstinspires.ftc.teamcode.game.tests.isolatedTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Flower Intake Tuner")
public class FlowerIntakeTuner extends LinearOpMode {
    @Override
    public void runOpMode() {
        Servo leftFlowerIntakeServo = hardwareMap.get(Servo.class, "leftFlowerIntake");
        Servo rightFlowerIntakeServo = hardwareMap.get(Servo.class, "rightFlowerIntake");

        rightFlowerIntakeServo.setDirection(Servo.Direction.REVERSE);

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.aWasPressed()) {
                leftFlowerIntakeServo.setPosition(leftFlowerIntakeServo.getPosition() - 0.01);

            }
            if (gamepad1.bWasPressed()) {
                leftFlowerIntakeServo.setPosition(leftFlowerIntakeServo.getPosition() + 0.01);
            }
            if (gamepad1.xWasPressed()) {
                rightFlowerIntakeServo.setPosition(rightFlowerIntakeServo.getPosition() - 0.01);
            }
            if (gamepad1.yWasPressed()) {
                rightFlowerIntakeServo.setPosition(rightFlowerIntakeServo.getPosition() + 0.01);
            }

            telemetry.addData("Left Flower Intake Servo", leftFlowerIntakeServo.getPosition());
            telemetry.addData("Right Flower Intake Servo", rightFlowerIntakeServo.getPosition());

            telemetry.update();
        }
    }
}
