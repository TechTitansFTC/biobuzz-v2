package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FlowerSubsystem extends SubsystemBase {
    private final Telemetry telemetry;
    private final Servo leftServo;
    private final Servo rightServo;

    public FlowerSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        leftServo = hardwareMap.get(Servo.class, "leftFlowerIntake"); // port 2 servo control hub
        rightServo = hardwareMap.get(Servo.class, "rightFlowerIntake"); // port 2 servo expansion hub

        rightServo.setDirection(Servo.Direction.REVERSE);

        register();
    }

    @Override
    public void periodic() {
        telemetry.addData("flowerLeftServo", getLeftServoPosition());
        telemetry.addData("flowerRightServo", getRightServoPosition());
    }

    public void setLeftServoPosition(double position) {
        leftServo.setPosition(position);
    }

    public void setRightServoPosition(double position) {
        rightServo.setPosition(position);
    }

    public double getLeftServoPosition() {
        return leftServo.getPosition();
    }

    public double getRightServoPosition() {
        return rightServo.getPosition();
    }
}
