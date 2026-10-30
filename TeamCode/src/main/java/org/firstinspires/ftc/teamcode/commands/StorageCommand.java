package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.util.Constants.INTAKE_MOTOR_POWER;
import static org.firstinspires.ftc.teamcode.util.Constants.REVERSE_INTAKE_MOTOR_POWER;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.StorageSubsystem;

public class StorageCommand extends CommandBase {
    private final StorageSubsystem storageSubsystem;
    private final IntakeSubsystem intakeSubsystem;

    // TODO: add boolean for out take state

    public StorageCommand(StorageSubsystem storageSubsystem, IntakeSubsystem intakeSubsystem) {
        this.storageSubsystem = storageSubsystem;
        this.intakeSubsystem = intakeSubsystem;

        addRequirements(storageSubsystem, intakeSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        int count = 0;
        if (storageSubsystem.getSensor1Color() != StorageSubsystem.GamePieceColor.NONE) count++;
        if (storageSubsystem.getSensor2Value()) count++;
        if (storageSubsystem.getSensor3Color() != StorageSubsystem.GamePieceColor.NONE) count++;
        if (storageSubsystem.getSensor4Value()) count++;
        if (storageSubsystem.getSensor5Value()) count++;
        if (count < 4) {
            intakeSubsystem.setMotorPower(INTAKE_MOTOR_POWER);
        } else if (count == 4) {
            intakeSubsystem.setMotorPower(-REVERSE_INTAKE_MOTOR_POWER);
        } else {
//            intakeSubsystem.setMotorPower(); // TODO: go to out take
        }
    }
}
