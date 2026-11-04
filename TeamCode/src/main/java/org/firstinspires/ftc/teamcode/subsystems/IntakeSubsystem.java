package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeSubsystem extends SubsystemBase {
    private final DcMotor intakeMotor;
    private final Servo flowerIntakeServoLeft;
    private final Servo flowerIntakeServoRight;
    private final Telemetry telemetry;

    public IntakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intake"); // port 2 motor expansion hub
        flowerIntakeServoLeft = hardwareMap.get(Servo.class, "leftFlowerIntake"); // port 2 servo control hub
        flowerIntakeServoRight = hardwareMap.get(Servo.class, "rightFlowerIntake"); // port 2 servo expansion hub

        this.telemetry = telemetry;

        register();
    }

    @Override
    public void periodic() {
        telemetry.addData("Intake Motor Power", getMotorPower());
    }

    public void setMotorPower(double power) {
        intakeMotor.setPower(power);
    }

    public double getMotorPower() {
        return intakeMotor.getPower();
    }
}
