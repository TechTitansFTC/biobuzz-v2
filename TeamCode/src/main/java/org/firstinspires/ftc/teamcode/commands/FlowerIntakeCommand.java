package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.util.Constants.FLOWER_LEFT_SERVO_UP;
import static org.firstinspires.ftc.teamcode.util.Constants.FLOWER_LEFT_SERVO_DOWN;
import static org.firstinspires.ftc.teamcode.util.Constants.FLOWER_RIGHT_SERVO_UP;
import static org.firstinspires.ftc.teamcode.util.Constants.FLOWER_RIGHT_SERVO_DOWN;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.FlowerSubsystem;

public class FlowerIntakeCommand extends CommandBase {
    private final FlowerSubsystem flowerSubsystem;

    public FlowerIntakeCommand(FlowerSubsystem flowerSubsystem) {
        this.flowerSubsystem = flowerSubsystem;

        addRequirements(flowerSubsystem);
    }

    @Override
    public void initialize() {
        flowerSubsystem.setLeftServoPosition(FLOWER_LEFT_SERVO_DOWN);
        flowerSubsystem.setRightServoPosition(FLOWER_RIGHT_SERVO_DOWN);
    }

    @Override
    public void end(boolean interrupted) {
        flowerSubsystem.setLeftServoPosition(FLOWER_LEFT_SERVO_UP);
        flowerSubsystem.setRightServoPosition(FLOWER_RIGHT_SERVO_UP);
    }
}
