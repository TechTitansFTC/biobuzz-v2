package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeSubsystem extends SubsystemBase {
    private final DcMotor intakeMotor;
    private final Telemetry telemetry;

    public IntakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        this.telemetry = telemetry;
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
