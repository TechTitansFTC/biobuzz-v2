package org.firstinspires.ftc.teamcode.game;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ScoringSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.StorageSubsystem;
import org.firstinspires.ftc.teamcode.util.Constants;

@Configurable
@TeleOp(name="PollenFlywheelAndTransfer")
public class PollenFlywheelAndTransfer extends LinearOpMode {
    public static double pollenMotorPower = 0.6;

    @Override
    public void runOpMode() {
        ScoringSubsystem scoringSubsystem = new ScoringSubsystem(hardwareMap, telemetry);
        StorageSubsystem storageSubsystem = new StorageSubsystem(hardwareMap, telemetry);

        waitForStart();
        while (opModeIsActive()) {
            scoringSubsystem.setPollenTurretMotorPower(pollenMotorPower);
            storageSubsystem.setTransferMotorPower(Constants.TRANSFER_MOTOR_POWER);
        }
    }
}
