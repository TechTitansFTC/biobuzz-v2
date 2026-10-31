package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.util.Constants.NECTAR_TURRET_MOTOR_CONTROLLER;
import static org.firstinspires.ftc.teamcode.util.Constants.NECTAR_TURRET_SERVO_CONTROLLER;
import static org.firstinspires.ftc.teamcode.util.Constants.POLLEN_TURRET_MOTOR_CONTROLLER;
import static org.firstinspires.ftc.teamcode.util.Constants.POLLEN_TURRET_SERVO_CONTROLLER;
import static org.firstinspires.ftc.teamcode.util.Constants.TEAM;

import com.pedropathing.follower.Follower;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ScoringSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.StorageSubsystem;
import org.firstinspires.ftc.teamcode.util.PIDController;

public class ScoringCommand extends CommandBase {
    private final ScoringSubsystem scoringSubsystem;
    private final StorageSubsystem storageSubsystem;
    private final Follower follower;
    private enum ScoringRoutine {FIND_ANGLE, ALIGN_IMU, ALIGN_LIMELIGHT, ALIGN_SECOND_TURRET, SET_MOTOR_POWER, SHOOT};
    private ScoringRoutine scoringState;
    private boolean endNow;
    private double angle;
    private final PIDController pollenTurretServoController;
    private final PIDController nectarTurretServoController;
    private final PIDController pollenTurretMotorController;
    private final PIDController nectarTurretMotorController;

    public ScoringCommand(ScoringSubsystem scoringSubsystem, StorageSubsystem storageSubsystem, Follower follower) {
        this.scoringSubsystem = scoringSubsystem;
        this.storageSubsystem = storageSubsystem;
        this.follower = follower;

        scoringState = ScoringRoutine.FIND_ANGLE;
        endNow = false;
        angle = 0;

        pollenTurretServoController = new PIDController(POLLEN_TURRET_SERVO_CONTROLLER[0], POLLEN_TURRET_SERVO_CONTROLLER[1],
                POLLEN_TURRET_SERVO_CONTROLLER[2]);
        nectarTurretServoController = new PIDController(NECTAR_TURRET_SERVO_CONTROLLER[0], NECTAR_TURRET_SERVO_CONTROLLER[1],
                NECTAR_TURRET_SERVO_CONTROLLER[2]);
        pollenTurretMotorController = new PIDController(POLLEN_TURRET_MOTOR_CONTROLLER[0], POLLEN_TURRET_MOTOR_CONTROLLER[1],
                POLLEN_TURRET_MOTOR_CONTROLLER[2]);
        nectarTurretMotorController = new PIDController(NECTAR_TURRET_MOTOR_CONTROLLER[0], NECTAR_TURRET_MOTOR_CONTROLLER[1],
                NECTAR_TURRET_MOTOR_CONTROLLER[2]);

        addRequirements(scoringSubsystem, storageSubsystem);
    }

    @Override
    public void initialize() {
        scoringState = ScoringRoutine.FIND_ANGLE;
        endNow = false;
        angle = 0;
    }

    @Override
    public void execute() {
        switch (scoringState) {
            case FIND_ANGLE:
                double currX = follower.pose().x() ;
                double currY = follower.pose().y();
                double currHeading = convertHeading(Math.toDegrees(follower.pose().heading()));
                double targetY = 0;
                double targetX = 57;
                if (currY < 55) targetY = 55;
                else if (currY > 89) targetY = 89;
                else endNow = true;
                if (TEAM.equals("B")) {
                    targetX = 84;
                }

                double xDiff = targetX - currX;
                double yDiff = targetY - currY;

                angle = Math.toDegrees(Math.atan2(xDiff, yDiff));
                angle -= currHeading;
                pollenTurretServoController.setTarget(angle);
                nectarTurretServoController.setTarget(angle);
                break;
            case ALIGN_IMU:
                double currentValue;

        }
    }

    @Override
    public boolean isFinished() {
        return endNow;
    }

    public double convertHeading(double beforeDegrees) {
        double after = 90.0 - beforeDegrees;

        while (after > 180.0) {
            after -= 360.0;
        }
        while (after <= -180.0) {
            after += 360.0;
        }

        return after;
    }
}
