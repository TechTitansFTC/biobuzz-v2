package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.util.Constants.ALIGN_APRIL_TAG_TIME;
import static org.firstinspires.ftc.teamcode.util.Constants.ALIGN_IMU_TIME;
import static org.firstinspires.ftc.teamcode.util.Constants.LEFT_NECTAR_GATE_CLOSE_POSITION;
import static org.firstinspires.ftc.teamcode.util.Constants.LEFT_NECTAR_GATE_OPEN_POSITION;
import static org.firstinspires.ftc.teamcode.util.Constants.NECTAR_TURRET_MOTOR_CONTROLLER;
import static org.firstinspires.ftc.teamcode.util.Constants.NECTAR_TURRET_SERVO_CONTROLLER;
import static org.firstinspires.ftc.teamcode.util.Constants.NECTAR_TURRET_VELOCITY_MODEL;
import static org.firstinspires.ftc.teamcode.util.Constants.POLLEN_MOTOR_KD;
import static org.firstinspires.ftc.teamcode.util.Constants.POLLEN_MOTOR_KF;
import static org.firstinspires.ftc.teamcode.util.Constants.POLLEN_MOTOR_KI;
import static org.firstinspires.ftc.teamcode.util.Constants.POLLEN_MOTOR_KP;
import static org.firstinspires.ftc.teamcode.util.Constants.POLLEN_SERVO_KD;
import static org.firstinspires.ftc.teamcode.util.Constants.POLLEN_SERVO_KF;
import static org.firstinspires.ftc.teamcode.util.Constants.POLLEN_SERVO_KI;
import static org.firstinspires.ftc.teamcode.util.Constants.POLLEN_SERVO_KP;
import static org.firstinspires.ftc.teamcode.util.Constants.POLLEN_TURRET_MOTOR_CONTROLLER;
import static org.firstinspires.ftc.teamcode.util.Constants.POLLEN_TURRET_SERVO_CONTROLLER;
import static org.firstinspires.ftc.teamcode.util.Constants.POLLEN_TURRET_VELOCITY_MODEL;
import static org.firstinspires.ftc.teamcode.util.Constants.TEAM;
import static org.firstinspires.ftc.teamcode.util.Constants.TRANSFER_MOTOR_POWER;
import static org.firstinspires.ftc.teamcode.util.Constants.POLLEN_GATE_CLOSE_POSITION;
import static org.firstinspires.ftc.teamcode.util.Constants.POLLEN_GATE_OPEN_POSITION;
import static org.firstinspires.ftc.teamcode.util.Constants.SHOOTING_TIME;

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
    private enum ScoringRoutine {CALCULATE, ALIGN_IMU, ALIGN_APRIL_TAGS, SHOOT, END};
    private final ElapsedTime stateTimer;
    private ScoringRoutine scoringState;
    private boolean endNow;
    private final PIDController pollenTurretServoController;
    private final PIDController pollenTurretMotorController;
    private final ElapsedTime controllerTimer;

    private double pollenTurretServoPower;
    private double nectarTurretServoPower;
    private double pollenTurretMotorPower;
    private double nectarTurretMotorPower;
    private double lateralDistance;

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
        lateralDistance = 0;

        pollenTurretServoController = new PIDController(POLLEN_TURRET_SERVO_CONTROLLER[0], POLLEN_TURRET_SERVO_CONTROLLER[1],
                POLLEN_TURRET_SERVO_CONTROLLER[2], POLLEN_TURRET_SERVO_CONTROLLER[3]);
        pollenTurretMotorController = new PIDController(POLLEN_TURRET_MOTOR_CONTROLLER[0], POLLEN_TURRET_MOTOR_CONTROLLER[1],
                POLLEN_TURRET_MOTOR_CONTROLLER[2], POLLEN_TURRET_MOTOR_CONTROLLER[3]);

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
        // Re-read the individual constants every loop so Panels edits apply live.
        pollenTurretServoController.setConstants(POLLEN_SERVO_KP, POLLEN_SERVO_KI, POLLEN_SERVO_KD, POLLEN_SERVO_KF);
        pollenTurretMotorController.setConstants(POLLEN_MOTOR_KP, POLLEN_MOTOR_KI, POLLEN_MOTOR_KD, POLLEN_MOTOR_KF);

        switch (scoringState) {
            case CALCULATE:
                double currX = follower.pose().x() ;
                double currY = follower.pose().y();
                double currHeading = convertHeading(Math.toDegrees(follower.pose().heading()));
                double targetY = 0;
                double targetX = 57;

                if (currY < 55) targetY = 55;
                else if (currY > 89) targetY = 89;
                else  {
                    endNow = true; // Cancels command if robot is not in scoring zone
                    return;
                }

                if (TEAM.equals("B")) {
                    targetX = 84;
                }

                double xDiff = targetX - currX;
                double yDiff = targetY - currY;

                double angle = Math.toDegrees(Math.atan2(xDiff, yDiff));
                angle = normalizeDegrees(angle - currHeading);

                lateralDistance = Math.sqrt(Math.pow(xDiff, 2) + Math.pow(yDiff, 2));
                double pollenTurretTargetRPM = POLLEN_TURRET_VELOCITY_MODEL(lateralDistance);

                pollenTurretServoController.setTarget(angle);
                pollenTurretMotorController.setTarget(pollenTurretTargetRPM);
                controllerTimer.reset();
                stateTimer.reset();

                scoringState = ScoringRoutine.ALIGN_IMU;
                break;
            case ALIGN_IMU:
                double currentValuePollenTurretServo = scoringSubsystem.getPollenTurretAngleDegrees();

                pollenTurretServoPower = pollenTurretServoController.calculateOutput(currentValuePollenTurretServo, controllerTimer.seconds());

                if (stateTimer.seconds() >= ALIGN_IMU_TIME) {
                    pollenTurretServoController.setTarget(0);
                    stateTimer.reset();
                    scoringState = ScoringRoutine.ALIGN_APRIL_TAGS;
                }


                break;
            case ALIGN_APRIL_TAGS:
                double offsetDegreesPollen = scoringSubsystem.getBearing();

                pollenTurretServoPower = pollenTurretServoController.calculateOutput(offsetDegreesPollen, controllerTimer.seconds());

                if (stateTimer.seconds() >= ALIGN_APRIL_TAG_TIME) {
                    stateTimer.reset();
                    storageSubsystem.setPollenGateServoPosition(POLLEN_GATE_OPEN_POSITION);
                    storageSubsystem.setTransferMotorPower(TRANSFER_MOTOR_POWER);
                    scoringState = ScoringRoutine.SHOOT;
                }


                break;
            case SHOOT:
                double offsetDegreesPollen2 = scoringSubsystem.getBearing();

                pollenTurretServoPower = pollenTurretServoController.calculateOutput(offsetDegreesPollen2, controllerTimer.seconds());

                if (stateTimer.seconds() >= SHOOTING_TIME) {
                    scoringState = ScoringRoutine.END;
                    stateTimer.reset();
                }

                break;
            case END:
                storageSubsystem.setPollenGateServoPosition(POLLEN_GATE_CLOSE_POSITION);
                storageSubsystem.setTransferMotorPower(0);
                break;
        }

        double currentValuePollenTurretMotor = scoringSubsystem.getPollenTurretMotorRPM();
        double currentValueNectarTurretMotor = scoringSubsystem.getNectarTurretMotorRPM();


        pollenTurretMotorPower = pollenTurretMotorController.calculateOutput(currentValuePollenTurretMotor, controllerTimer.seconds());

        scoringSubsystem.setPollenTurretServoPower(pollenTurretServoPower);
        scoringSubsystem.setPollenTurretMotorPower(pollenTurretMotorPower);
        controllerTimer.reset();
    }

    @Override
    public boolean isFinished() {
        return endNow || scoringState == ScoringRoutine.END;
    }

    @Override
    public void end(boolean interrupted) {
        storageSubsystem.setPollenGateServoPosition(POLLEN_GATE_CLOSE_POSITION);
        storageSubsystem.setTransferMotorPower(0);
        scoringSubsystem.setPollenTurretServoPower(0);
        scoringSubsystem.setNectarTurretServoPower(0);
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



    public double normalizeDegrees(double degrees) {
        while (degrees > 180.0) {
            degrees -= 360.0;
        }
        while (degrees <= -180.0) {
            degrees += 360.0;
        }
        return degrees;
    }
}
