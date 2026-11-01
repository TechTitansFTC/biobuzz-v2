package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.util.Constants.ALIGN_IMU_TIME;
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
    private enum ScoringRoutine {CALCULATE, ALIGN_IMU, ALIGN_APRIL_TAGS, ALIGN_SECOND_TURRET, SET_MOTOR_POWER, SHOOT};
    private final ElapsedTime stateTimer;
    private ScoringRoutine scoringState;
    private boolean endNow;
    private final PIDController pollenTurretServoController;
    private final PIDController nectarTurretServoController;
    private final PIDController pollenTurretMotorController;
    private final PIDController nectarTurretMotorController;
    private final ElapsedTime controllerTimer;

    private double pollenTurretServoPower;
    private double nectarTurretServoPower;
    private double pollenTurretMotorPower;
    private double nectarTurretMotorPower;

    public ScoringCommand(ScoringSubsystem scoringSubsystem, StorageSubsystem storageSubsystem, Follower follower) {
        this.scoringSubsystem = scoringSubsystem;
        this.storageSubsystem = storageSubsystem;
        this.follower = follower;

        scoringState = ScoringRoutine.CALCULATE;
        endNow = false;

        controllerTimer = new ElapsedTime();
        stateTimer = new ElapsedTime();

        pollenTurretServoPower = 0;
        nectarTurretServoPower = 0;
        pollenTurretMotorPower = 0;
        nectarTurretMotorPower = 0;

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
        stateTimer.reset();
        endNow = false;
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
                else endNow = true; // Cancels command if robot is not in scoring zone

                if (TEAM.equals("B")) {
                    targetX = 84;
                }

                double xDiff = targetX - currX;
                double yDiff = targetY - currY;

                double angle = Math.toDegrees(Math.atan2(xDiff, yDiff));
                angle -= currHeading;

                double lateralDistance = Math.sqrt(Math.pow(xDiff, 2) + Math.pow(yDiff, 2));
                double pollenTurretTargetRPM = POLLEN_TURRET_VELOCITY_MODEL(lateralDistance);
                double nectarTurretTargetRPM = NECTAR_TURRET_VELOCITY_MODEL(lateralDistance);

                pollenTurretServoController.setTarget(angle);
                nectarTurretServoController.setTarget(angle);
                pollenTurretMotorController.setTarget(pollenTurretTargetRPM);
                nectarTurretMotorController.setTarget(nectarTurretTargetRPM);

                controllerTimer.reset();
                stateTimer.reset();

                scoringState = ScoringRoutine.ALIGN_IMU;
                break;
            case ALIGN_IMU:
                double currentValuePollenTurretServo = scoringSubsystem.getPollenTurretAngleDegrees() * ENCODER_TO_TURRET_RATIO;
                double currentValueNectarTurretServo = scoringSubsystem.getNectarTurretAngleDegrees() * ENCODER_TO_TURRET_RATIO; // TODO: Check if the ratio is correct
                double currentValuePollenTurretMotor = scoringSubsystem.getPollenTurretMotorRPM();
                double currentValueNectarTurretMotor = scoringSubsystem.getNectarTurretMotorRPM();

                pollenTurretServoPower = pollenTurretServoController.calculateOutput(currentValuePollenTurretServo, controllerTimer.seconds());
                nectarTurretServoPower = nectarTurretServoController.calculateOutput(currentValueNectarTurretServo, controllerTimer.seconds());
                pollenTurretMotorPower = pollenTurretMotorController.calculateOutput(currentValuePollenTurretMotor, controllerTimer.seconds());
                nectarTurretMotorPower = nectarTurretMotorController.calculateOutput(currentValueNectarTurretMotor, controllerTimer.seconds());

                scoringSubsystem.setPollenTurretServoPower(pollenTurretServoPower);
                scoringSubsystem.setNectarTurretServoPower(nectarTurretServoPower);
                scoringSubsystem.setPollenTurretMotorPower(pollenTurretMotorPower);
                scoringSubsystem.setNectarTurretMotorPower(nectarTurretMotorPower);

                if (stateTimer.seconds() >= ALIGN_IMU_TIME) {
                    stateTimer.reset();
                    scoringState = ScoringRoutine.ALIGN_APRIL_TAGS;
                }

                controllerTimer.reset();
                break;
            case ALIGN_APRIL_TAGS:
                pollenTurretServoController.setTarget(0);

                double offsetDegrees = scoringSubsystem.getBearing();


                break;
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
