package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.util.Constants.INTAKE_MOTOR_POWER;
import static org.firstinspires.ftc.teamcode.util.Constants.OUT_TAKE_TIME;
import static org.firstinspires.ftc.teamcode.util.Constants.REVERSE_INTAKE_MOTOR_POWER;
import static org.firstinspires.ftc.teamcode.util.Constants.REVERSE_TRANSFER_MOTOR_POWER;
import static org.firstinspires.ftc.teamcode.util.Constants.TEAM;
import static org.firstinspires.ftc.teamcode.util.Constants.TRANSFER_MOTOR_POWER;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.StorageSubsystem;

public class StorageCommand extends CommandBase {
    private final StorageSubsystem storageSubsystem;
    private final IntakeSubsystem intakeSubsystem;

    private boolean outTaking = false;
    private ElapsedTime outTakingTimer;

    public StorageCommand(StorageSubsystem storageSubsystem, IntakeSubsystem intakeSubsystem) {
        this.storageSubsystem = storageSubsystem;
        this.intakeSubsystem = intakeSubsystem;

        outTakingTimer = new ElapsedTime();

        addRequirements(storageSubsystem, intakeSubsystem);
    }

    @Override
    public void initialize() {
        outTaking = false;
        outTakingTimer.reset();
    }

    @Override
    public void execute() {
        if (outTaking) {
            if (outTakingTimer.seconds() >= OUT_TAKE_TIME) {
                outTaking = false;
            }
            return;
        }
        int count = 0;
        storageSubsystem.setTransferMotorPower(TRANSFER_MOTOR_POWER);
        if (storageSubsystem.getSensor1Color() != StorageSubsystem.GamePieceColor.NONE) count++;
        if ((storageSubsystem.getSensor1Color() == StorageSubsystem.GamePieceColor.RED && TEAM.equals("B")) || (storageSubsystem.getSensor1Color() == StorageSubsystem.GamePieceColor.BLUE && TEAM.equals("R"))) {
            intakeSubsystem.setMotorPower(-REVERSE_INTAKE_MOTOR_POWER);
            storageSubsystem.setTransferMotorPower(-REVERSE_TRANSFER_MOTOR_POWER);
            outTaking = true;
        }
        if (storageSubsystem.getSensor2Value()) count++;
        if (storageSubsystem.getSensor3Color() != StorageSubsystem.GamePieceColor.NONE) count++;
        if (storageSubsystem.getSensor4Value()) count++;
        if (storageSubsystem.getSensor5Value()) count++;
        if (count < 4) {
            intakeSubsystem.setMotorPower(INTAKE_MOTOR_POWER);
        } else if (count == 4) {
            intakeSubsystem.setMotorPower(-REVERSE_INTAKE_MOTOR_POWER);
            storageSubsystem.setTransferMotorPower(0);
        } else {
            outTakingTimer.reset();
            intakeSubsystem.setMotorPower(-REVERSE_INTAKE_MOTOR_POWER);
            storageSubsystem.setTransferMotorPower(-REVERSE_TRANSFER_MOTOR_POWER);
            outTaking = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setMotorPower(0);
        storageSubsystem.setTransferMotorPower(0);
    }
}
