package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ScoringSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.StorageSubsystem;

public class ScoringCommand extends CommandBase {
    private final ScoringSubsystem scoringSubsystem;
    private final StorageSubsystem storageSubsystem;

    private enum ScoringRoutine {ALIGN_IMU, ALIGN_LIMELIGHT, ALIGN_SECOND_TURRET};

    public ScoringCommand(ScoringSubsystem scoringSubsystem, StorageSubsystem storageSubsystem) {
        this.scoringSubsystem = scoringSubsystem;
        this.storageSubsystem = storageSubsystem;
    }

    @Override
    public void initialize() {

    }
}
