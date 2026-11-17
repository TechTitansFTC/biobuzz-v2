package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

public class DriveCommand extends CommandBase {
    private final MecanumDriveSubsystem drive;
    private final GamepadEx gamepad;

    public DriveCommand(GamepadEx gamepad, MecanumDriveSubsystem drive) {
        this.drive = drive;
        this.gamepad = gamepad;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        double x = gamepad.getLeftX();
        double y = gamepad.getLeftY();
        double rotation = gamepad.getRightX();

        drive.drive(x, y, rotation);
    }

    @Override
    public void end(boolean interrupted) { drive.drive(0, 0, 0); }
}
