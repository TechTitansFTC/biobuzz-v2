package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.util.Constants.REVERSE_INTAKE_MOTOR_POWER;
import static org.firstinspires.ftc.teamcode.util.Constants.REVERSE_TRANSFER_MOTOR_POWER;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.StorageSubsystem;

public class ReverseCommand extends CommandBase {
    private final StorageSubsystem storageSubsystem;
    private final IntakeSubsystem intakeSubsystem;

    public ReverseCommand(StorageSubsystem storageSubsystem, IntakeSubsystem intakeSubsystem) {
        this.storageSubsystem = storageSubsystem;
        this.intakeSubsystem = intakeSubsystem;

        addRequirements(storageSubsystem, intakeSubsystem);
    }

    @Override
    public void initialize() {
        storageSubsystem.setTransferMotorPower(-REVERSE_TRANSFER_MOTOR_POWER);
        intakeSubsystem.setMotorPower(-REVERSE_INTAKE_MOTOR_POWER);
    }

    @Override
    public void end(boolean inturrupted) {
        storageSubsystem.setTransferMotorPower(0);
        intakeSubsystem.setMotorPower(0);
    }
}
