package org.firstinspires.ftc.teamcode.game;

import static org.firstinspires.ftc.teamcode.util.Constants.*;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.commands.StorageCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.StorageSubsystem;
import org.firstinspires.ftc.teamcode.util.Constants;

@Configurable
@TeleOp(name = "1. Intake & Storage Tuner", group = "Tuners")
public class IntakeStorageTunerOpMode extends LinearOpMode {
    public static boolean RUN_AUTO_STORAGE_LOGIC = false;

    @Override
    public void runOpMode() {
        CommandScheduler.getInstance().reset();
        TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        IntakeSubsystem intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry);
        StorageSubsystem storageSubsystem = new StorageSubsystem(hardwareMap, telemetry);
        StorageCommand storageCommand = new StorageCommand(storageSubsystem, intakeSubsystem);

        boolean xWasPressed = false;

        waitForStart();

        while (opModeIsActive()) {
            // Toggle between Manual Control and Automated StorageCommand with Gamepad1 X
            if (gamepad1.x && !xWasPressed) {
                RUN_AUTO_STORAGE_LOGIC = !RUN_AUTO_STORAGE_LOGIC;
                if (RUN_AUTO_STORAGE_LOGIC) {
                    storageCommand.initialize();
                } else {
                    storageCommand.end(true);
                }
            }
            xWasPressed = gamepad1.x;

            if (RUN_AUTO_STORAGE_LOGIC) {
                storageCommand.execute();
            } else {
                // Manual Controls: Right Bumper = Intake, Left Bumper = Reverse/Outtake, Triggers = Transfer
                if (gamepad1.right_bumper) {
                    intakeSubsystem.setMotorPower(Constants.INTAKE_MOTOR_POWER);
                } else if (gamepad1.left_bumper) {
                    intakeSubsystem.setMotorPower(-Constants.REVERSE_INTAKE_MOTOR_POWER);
                } else {
                    intakeSubsystem.setMotorPower(0);
                }

                if (gamepad1.right_trigger > 0.1) {
                    storageSubsystem.setTransferMotorPower(Constants.TRANSFER_MOTOR_POWER);
                } else if (gamepad1.left_trigger > 0.1) {
                    storageSubsystem.setTransferMotorPower(-Constants.REVERSE_TRANSFER_MOTOR_POWER);
                } else {
                    storageSubsystem.setTransferMotorPower(0);
                }
            }

            // Panels Graph & Telemetry
            panelsTelemetry.addData("Mode (Press X to toggle)", RUN_AUTO_STORAGE_LOGIC ? "AUTO STORAGE COMMAND" : "MANUAL OVERRIDE");
            panelsTelemetry.addData("Sensor 1 Color", storageSubsystem.getSensor1Color());
            panelsTelemetry.addData("Sensor 1 Hue", storageSubsystem.getSensor1Hue());
            panelsTelemetry.addData("Sensor 1 Distance", storageSubsystem.getSensor1Distance());
            panelsTelemetry.addData("Sensor 2 Dist Triggered", storageSubsystem.getSensor2Value());
            panelsTelemetry.addData("Sensor 3 Color", storageSubsystem.getSensor3Color());
            panelsTelemetry.addData("Sensor 4 Dist Triggered", storageSubsystem.getSensor4Value());
            panelsTelemetry.addData("Sensor 5 Dist Triggered", storageSubsystem.getSensor5Value());
            panelsTelemetry.addData("Intake Power", intakeSubsystem.getMotorPower());
            panelsTelemetry.addData("Transfer Power", storageSubsystem.getTransferMotorPower());
            panelsTelemetry.addData("Nectar Gate Pos", storageSubsystem.getNectarGateServoPosition());
            panelsTelemetry.addData("Pollen Gate Pos", storageSubsystem.getPollenGateServoPosition());
            panelsTelemetry.update(telemetry);
        }
    }
}