package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.util.Constants.ALIGN_APRIL_TAG_TIME;
import static org.firstinspires.ftc.teamcode.util.Constants.ALIGN_IMU_TIME;
import static org.firstinspires.ftc.teamcode.util.Constants.ENCODER_TO_TURRET_RATIO;
import static org.firstinspires.ftc.teamcode.util.Constants.NECTAR_TURRET_MOTOR_CONTROLLER;
import static org.firstinspires.ftc.teamcode.util.Constants.NECTAR_TURRET_SERVO_CONTROLLER;
import static org.firstinspires.ftc.teamcode.util.Constants.NECTAR_TURRET_VELOCITY_MODEL;
import static org.firstinspires.ftc.teamcode.util.Constants.POLLEN_TURRET_MOTOR_CONTROLLER;
import static org.firstinspires.ftc.teamcode.util.Constants.POLLEN_TURRET_SERVO_CONTROLLER;
import static org.firstinspires.ftc.teamcode.util.Constants.POLLEN_TURRET_VELOCITY_MODEL;
import static org.firstinspires.ftc.teamcode.util.Constants.TEAM;
import static org.firstinspires.ftc.teamcode.util.Constants.TRANSFER_MOTOR_POWER;
import static org.firstinspires.ftc.teamcode.util.Constants.NECTAR_GATE_CLOSE_POSITION;
import static org.firstinspires.ftc.teamcode.util.Constants.NECTAR_GATE_OPEN_POSITION;
import static org.firstinspires.ftc.teamcode.util.Constants.POLLEN_GATE_CLOSE_POSITION;
import static org.firstinspires.ftc.teamcode.util.Constants.POLLEN_GATE_OPEN_POSITION;
import static org.firstinspires.ftc.teamcode.util.Constants.shootingTime;

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
    private final PIDController nectarTurretServoController;
    private final PIDController pollenTurretMotorController;
    private final PIDController nectarTurretMotorController;
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
        nectarTurretServoController = new PIDController(NECTAR_TURRET_SERVO_CONTROLLER[0], NECTAR_TURRET_SERVO_CONTROLLER[1],
                NECTAR_TURRET_SERVO_CONTROLLER[2], NECTAR_TURRET_SERVO_CONTROLLER[3]);
        pollenTurretMotorController = new PIDController(POLLEN_TURRET_MOTOR_CONTROLLER[0], POLLEN_TURRET_MOTOR_CONTROLLER[1],
                POLLEN_TURRET_MOTOR_CONTROLLER[2], POLLEN_TURRET_MOTOR_CONTROLLER[3]);
        nectarTurretMotorController = new PIDController(NECTAR_TURRET_MOTOR_CONTROLLER[0], NECTAR_TURRET_MOTOR_CONTROLLER[1],
                NECTAR_TURRET_MOTOR_CONTROLLER[2], NECTAR_TURRET_MOTOR_CONTROLLER[3]);

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
                double currentValuePollenTurretServo = scoringSubsystem.getPollenTurretAngleDegrees();
                double currentValueNectarTurretServo = scoringSubsystem.getNectarTurretAngleDegrees(); // TODO: Check if the ratio is correct

                pollenTurretServoPower = pollenTurretServoController.calculateOutput(currentValuePollenTurretServo, controllerTimer.seconds());
                nectarTurretServoPower = nectarTurretServoController.calculateOutput(currentValueNectarTurretServo, controllerTimer.seconds());

                if (stateTimer.seconds() >= ALIGN_IMU_TIME) {
                    pollenTurretServoController.setTarget(0);
                    nectarTurretServoController.setTarget(getSecondTurretBearing(lateralDistance, 5.669, scoringSubsystem.getBearing()));
                    stateTimer.reset();
                    scoringState = ScoringRoutine.ALIGN_APRIL_TAGS;
                }


                break;
            case ALIGN_APRIL_TAGS:
                double offsetDegreesPollen = scoringSubsystem.getBearing();
                double offsetDegreesNectar = scoringSubsystem.getPollenTurretAngleDegrees();

                pollenTurretServoPower = pollenTurretServoController.calculateOutput(offsetDegreesPollen, controllerTimer.seconds());
                nectarTurretServoPower = nectarTurretServoController.calculateOutput(offsetDegreesNectar, controllerTimer.seconds());

                if (stateTimer.seconds() >= ALIGN_APRIL_TAG_TIME) {
                    stateTimer.reset();
                    scoringSubsystem.setPollenGateServoPosition(POLLEN_GATE_OPEN_POSITION);
                    scoringSubsystem.setNectarGateServoPosition(NECTAR_GATE_OPEN_POSITION);
                    storageSubsystem.setTransferMotorPower(TRANSFER_MOTOR_POWER);
                    scoringState = ScoringRoutine.SHOOT;
                }


                break;
            case SHOOT:
                double offsetDegreesPollen2 = scoringSubsystem.getBearing();
                double offsetDegreesNectar2 = getSecondTurretBearing(lateralDistance, 5.669, offsetDegreesPollen2);

                pollenTurretServoPower = pollenTurretServoController.calculateOutput(offsetDegreesPollen2, controllerTimer.seconds());
                nectarTurretServoPower = nectarTurretServoController.calculateOutput(offsetDegreesNectar2, controllerTimer.seconds());

                if (stateTimer.seconds() >= shootingTime) {
                    scoringState = ScoringRoutine.END;
                    stateTimer.reset();
                }

                break;
            case END:
                scoringSubsystem.setPollenGateServoPosition(POLLEN_GATE_CLOSE_POSITION);
                scoringSubsystem.setNectarGateServoPosition(NECTAR_GATE_CLOSE_POSITION);
                storageSubsystem.setTransferMotorPower(0);
                break;
        }

        double currentValuePollenTurretMotor = scoringSubsystem.getPollenTurretMotorRPM();
        double currentValueNectarTurretMotor = scoringSubsystem.getNectarTurretMotorRPM();


        pollenTurretMotorPower = pollenTurretMotorController.calculateOutput(currentValuePollenTurretMotor, controllerTimer.seconds());
        nectarTurretMotorPower = nectarTurretMotorController.calculateOutput(currentValueNectarTurretMotor, controllerTimer.seconds());

        scoringSubsystem.setPollenTurretServoPower(pollenTurretServoPower);
        scoringSubsystem.setNectarTurretServoPower(nectarTurretServoPower);
        scoringSubsystem.setPollenTurretMotorPower(pollenTurretMotorPower);
        scoringSubsystem.setNectarTurretMotorPower(nectarTurretMotorPower);
        controllerTimer.reset();
    }

    @Override
    public boolean isFinished() {
        return endNow || scoringState == ScoringRoutine.END;
    }

    @Override
    public void end(boolean interrupted) {
        scoringSubsystem.setPollenGateServoPosition(POLLEN_GATE_CLOSE_POSITION);
        scoringSubsystem.setNectarGateServoPosition(NECTAR_GATE_CLOSE_POSITION);
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

    public double getSecondTurretBearing(double x, double y, double thetaDegrees) {
        // Convert the first turret's bearing to radians for Java Math functions
        double theta = Math.toRadians(thetaDegrees);

        // Calculate the components from the offset formula
        double xComponent = x * Math.sin(theta);
        double yComponent = y - (x * Math.cos(theta));

        // Use atan2 with (X, Y) to match your custom 0=forward coordinate system
        double alpha = Math.atan2(xComponent, yComponent);

        // Convert the result back to degrees
        return Math.toDegrees(alpha);
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
