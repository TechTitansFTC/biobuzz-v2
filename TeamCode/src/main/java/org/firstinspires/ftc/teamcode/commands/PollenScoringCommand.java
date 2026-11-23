package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.util.Constants.INTAKE_MOTOR_POWER;
import static org.firstinspires.ftc.teamcode.util.Constants.POLLEN_ALIGN_TIMEOUT;
import static org.firstinspires.ftc.teamcode.util.Constants.POLLEN_GATE_CLOSE_POSITION;
import static org.firstinspires.ftc.teamcode.util.Constants.POLLEN_GATE_OPEN_POSITION;
import static org.firstinspires.ftc.teamcode.util.Constants.POLLEN_TURRET_TAG_KD;
import static org.firstinspires.ftc.teamcode.util.Constants.POLLEN_TURRET_TAG_KF;
import static org.firstinspires.ftc.teamcode.util.Constants.POLLEN_TURRET_TAG_KI;
import static org.firstinspires.ftc.teamcode.util.Constants.POLLEN_TURRET_TAG_KP;
import static org.firstinspires.ftc.teamcode.util.Constants.POLLEN_TURRET_TAG_REVERSED;
import static org.firstinspires.ftc.teamcode.util.Constants.SHOOTING_TIME;
import static org.firstinspires.ftc.teamcode.util.Constants.TRANSFER_MOTOR_POWER;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ScoringSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.StorageSubsystem;
import org.firstinspires.ftc.teamcode.util.PIDController;

/**
 * Shoots pollen. The flywheel is NOT controlled here: ScoringSubsystem.periodic() keeps it at
 * POLLEN_SHOOTER_TARGET_RPM for the whole OpMode, so this command never stops it.
 *
 *   ALIGN - turret tracks the AprilTag; gate closed, transfer off. Waits until the flywheel is
 *           within +/- POLLEN_SHOOTER_RPM_TOLERANCE.
 *   SHOOT - open the gate and run the transfer for SHOOTING_TIME while the turret keeps tracking.
 *   DONE  - command ends; end() closes the gate and stops the transfer and turret servo.
 * If the flywheel isn't up to speed within POLLEN_ALIGN_TIMEOUT, it ends without shooting.
 */
public class PollenScoringCommand extends CommandBase {
    private static final double NO_TAG = 361.0; // ScoringSubsystem.getBearing() returns this when no tag is seen

    private enum State { ALIGN, SHOOT, DONE }

    private final ScoringSubsystem scoringSubsystem;
    private final StorageSubsystem storageSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final PIDController turretController;
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime loopTimer = new ElapsedTime();

    private State state = State.ALIGN;

    public PollenScoringCommand(ScoringSubsystem scoringSubsystem, StorageSubsystem storageSubsystem, IntakeSubsystem intakeSubsystem) {
        this.scoringSubsystem = scoringSubsystem;
        this.storageSubsystem = storageSubsystem;
        this.intakeSubsystem = intakeSubsystem;

        turretController = new PIDController(POLLEN_TURRET_TAG_KP, POLLEN_TURRET_TAG_KI, POLLEN_TURRET_TAG_KD, POLLEN_TURRET_TAG_KF);

        addRequirements(scoringSubsystem, storageSubsystem);
    }

    @Override
    public void initialize() {
        state = State.ALIGN;
        stateTimer.reset();
        loopTimer.reset();

        turretController.setTarget(0); // drive the tag bearing to 0 = turret pointed at the tag

        // Gate stays closed until the flywheel is up to speed.
        storageSubsystem.setPollenGateServoPosition(POLLEN_GATE_CLOSE_POSITION);
        storageSubsystem.setTransferMotorPower(0);
        intakeSubsystem.setMotorPower(INTAKE_MOTOR_POWER);
    }

    @Override
    public void execute() {
        double dt = loopTimer.seconds();
        loopTimer.reset();

        updateTurret(dt);

        switch (state) {
            case ALIGN:
                storageSubsystem.setPollenGateServoPosition(POLLEN_GATE_CLOSE_POSITION);
                storageSubsystem.setTransferMotorPower(0);
                intakeSubsystem.setMotorPower(INTAKE_MOTOR_POWER);


                if (stateTimer.seconds() >= POLLEN_ALIGN_TIMEOUT) {
                    storageSubsystem.setPollenGateServoPosition(POLLEN_GATE_OPEN_POSITION);
                    storageSubsystem.setTransferMotorPower(TRANSFER_MOTOR_POWER);
                    intakeSubsystem.setMotorPower(INTAKE_MOTOR_POWER);
                    stateTimer.reset();
                    state = State.SHOOT;
                }

                break;

            case SHOOT:
                storageSubsystem.setPollenGateServoPosition(POLLEN_GATE_OPEN_POSITION);
                storageSubsystem.setTransferMotorPower(TRANSFER_MOTOR_POWER);
                intakeSubsystem.setMotorPower(INTAKE_MOTOR_POWER);

                if (stateTimer.seconds() >= SHOOTING_TIME) {
                    state = State.DONE;
                }
                break;

            case DONE:
                break;
        }
    }

    /** Points the turret at the AprilTag, same math as CameraTest. */
    private void updateTurret(double dt) {
        turretController.setConstants(POLLEN_TURRET_TAG_KP, POLLEN_TURRET_TAG_KI, POLLEN_TURRET_TAG_KD, POLLEN_TURRET_TAG_KF);

        double bearing = scoringSubsystem.getBearing();
        if (bearing == NO_TAG) {
            scoringSubsystem.setPollenTurretServoPower(0); // no tag in view: hold still
            return;
        }

        double angle = -bearing;
        double output = turretController.calculateOutput(angle, dt);
        if (POLLEN_TURRET_TAG_REVERSED) output = -output;

        scoringSubsystem.setPollenTurretServoPower(Range.clip(output, -1, 1));
    }

    @Override
    public boolean isFinished() {
        return state == State.DONE;
    }

    @Override
    public void end(boolean interrupted) {
        // Flywheel is intentionally left running (ScoringSubsystem keeps it at speed).
        storageSubsystem.setPollenGateServoPosition(POLLEN_GATE_CLOSE_POSITION);
        storageSubsystem.setTransferMotorPower(0);
        scoringSubsystem.setPollenTurretServoPower(0);
    }
}
