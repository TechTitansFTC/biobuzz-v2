package org.firstinspires.ftc.teamcode.game;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Servo Test")
public class ServoTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Servo testServo = hardwareMap.get(Servo.class, "servo");

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.aWasPressed()) {
                testServo.setPosition(testServo.getPosition()+0.01);
            }
            if (gamepad1.bWasPressed()) {
                testServo.setPosition(testServo.getPosition()-0.01);
            }
            telemetry.addData("Servo Position", testServo.getPosition());

            telemetry.update();
        }
    }
}
