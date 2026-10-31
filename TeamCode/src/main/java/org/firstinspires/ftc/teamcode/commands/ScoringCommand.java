package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.util.Constants.ENCODER_TO_TURRET_RATIO;
import static org.firstinspires.ftc.teamcode.util.Constants.NECTAR_TURRET_MOTOR_CONTROLLER;
import static org.firstinspires.ftc.teamcode.util.Constants.NECTAR_TURRET_SERVO_CONTROLLER;
import static org.firstinspires.ftc.teamcode.util.Constants.NECTAR_TURRET_VELOCITY_MODEL;
import static org.firstinspires.ftc.teamcode.util.Constants.POLLEN_TURRET_MOTOR_CONTROLLER;
import static org.firstinspires.ftc.teamcode.util.Constants.POLLEN_TURRET_SERVO_CONTROLLER;
import static org.firstinspires.ftc.teamcode.util.Constants.POLLEN_TURRET_VELOCITY_MODEL;
import static org.firstinspires.ftc.teamcode.util.Constants.TEAM;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ScoringSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.StorageSubsystem;
import org.firstinspires.ftc.teamcode.util.PIDController;

public class ScoringCommand extends CommandBase {
    private final ScoringSubsystem scoringSubsystem;
    private final StorageSubsystem storageSubsystem;
    private final Follower follower;
    private enum ScoringRoutine {CALCULATE, ALIGN_IMU, ALIGN_LIMELIGHT, ALIGN_SECOND_TURRET, SET_MOTOR_POWER, SHOOT};
    private ScoringRoutine scoringState;
    private boolean endNow;
    private double angle;
    private final PIDController pollenTurretServoController;
    private final PIDController nectarTurretServoController;
    private final PIDController pollenTurretMotorController;
    private final PIDController nectarTurretMotorController;
    private final ElapsedTime turretServoControllerTimer;
    private final ElapsedTime turretMotorControllerTimer;

    public ScoringCommand(ScoringSubsystem scoringSubsystem, StorageSubsystem storageSubsystem, Follower follower) {
        this.scoringSubsystem = scoringSubsystem;
        this.storageSubsystem = storageSubsystem;
        this.follower = follower;

        scoringState = ScoringRoutine.CALCULATE;
        endNow = false;
        angle = 0;

        turretServoControllerTimer = new ElapsedTime();
        turretMotorControllerTimer = new ElapsedTime();

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
        scoringState = ScoringRoutine.CALCULATE;
        endNow = false;
        angle = 0;
    }

    @Override
    public void execute() {
        switch (scoringState) {
            case CALCULATE:
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

                double lateralDistance = Math.sqrt(Math.pow(xDiff, 2) + Math.pow(yDiff, 2));
                double pollenTurretTargetRPM = POLLEN_TURRET_VELOCITY_MODEL(lateralDistance);
                double nectarTurretTargetRPM = NECTAR_TURRET_VELOCITY_MODEL(lateralDistance);

                pollenTurretServoController.setTarget(angle);
                nectarTurretServoController.setTarget(angle);
                pollenTurretMotorController.setTarget(pollenTurretTargetRPM);
                nectarTurretMotorController.setTarget(nectarTurretTargetRPM);

                turretServoControllerTimer.reset();
                turretMotorControllerTimer.reset();

                scoringState = ScoringRoutine.ALIGN_IMU;
                break;
            case ALIGN_IMU:
                double currentValuePollenTurretServo = scoringSubsystem.getPollenTurretAngleDegrees() * ENCODER_TO_TURRET_RATIO;
//                double currentValue

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
