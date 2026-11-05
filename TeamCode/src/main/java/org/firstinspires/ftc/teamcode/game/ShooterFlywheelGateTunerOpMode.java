package org.firstinspires.ftc.teamcode.game;

import static org.firstinspires.ftc.teamcode.util.Constants.*;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.subsystems.ScoringSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.StorageSubsystem;
import org.firstinspires.ftc.teamcode.util.PIDController;

@Configurable
@TeleOp(name = "3. Shooter RPM & Gate Tuner", group = "Tuners")
public class ShooterFlywheelGateTunerOpMode extends LinearOpMode {
    public static boolean USE_DISTANCE_MODEL = false;
    public static double TEST_DISTANCE_INCHES = 60.0;
    public static double DIRECT_TARGET_POLLEN_RPM = 0.0;
    public static double DIRECT_TARGET_NECTAR_RPM = 0.0;

    @Override
    public void runOpMode() {
        CommandScheduler.getInstance().reset();
        TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        ScoringSubsystem scoringSubsystem = new ScoringSubsystem(hardwareMap, telemetry);
        StorageSubsystem storageSubsystem = new StorageSubsystem(hardwareMap, telemetry);

        PIDController pollenMotorPID = new PIDController(POLLEN_MOTOR_KP, POLLEN_MOTOR_KI, POLLEN_MOTOR_KD, POLLEN_MOTOR_KF);
        PIDController nectarMotorPID = new PIDController(NECTAR_MOTOR_KP, NECTAR_MOTOR_KI, NECTAR_MOTOR_KD, NECTAR_MOTOR_KF);

        ElapsedTime loopTimer = new ElapsedTime();
        boolean flywheelsEnabled = false;
        boolean aWasPressed = false;

        waitForStart();
        loopTimer.reset();

        while (opModeIsActive()) {
            double dt = loopTimer.seconds();
            loopTimer.reset();

            // Update PID constants live from Panels
            pollenMotorPID.setConstants(POLLEN_MOTOR_KP, POLLEN_MOTOR_KI, POLLEN_MOTOR_KD, POLLEN_MOTOR_KF);
            nectarMotorPID.setConstants(NECTAR_MOTOR_KP, NECTAR_MOTOR_KI, NECTAR_MOTOR_KD, NECTAR_MOTOR_KF);

            // Toggle Flywheels ON/OFF with Gamepad1 A
            if (gamepad1.a && !aWasPressed) {
                flywheelsEnabled = !flywheelsEnabled;
            }
            aWasPressed = gamepad1.a;

            // Hold Gamepad1 B to open gates and run transfer motor (Simulate SHOOT state)
            if (gamepad1.b) {
                scoringSubsystem.setPollenGateServoPosition(POLLEN_GATE_OPEN_POSITION);
                scoringSubsystem.setNectarGateServoPosition(NECTAR_GATE_OPEN_POSITION);
                storageSubsystem.setTransferMotorPower(TRANSFER_MOTOR_POWER);
            } else {
                scoringSubsystem.setPollenGateServoPosition(POLLEN_GATE_CLOSE_POSITION);
                scoringSubsystem.setNectarGateServoPosition(NECTAR_GATE_CLOSE_POSITION);
                storageSubsystem.setTransferMotorPower(0);
            }

            double targetPollenRPM = USE_DISTANCE_MODEL
                    ? POLLEN_TURRET_VELOCITY_MODEL(TEST_DISTANCE_INCHES)
                    : DIRECT_TARGET_POLLEN_RPM;
            double targetNectarRPM = USE_DISTANCE_MODEL
                    ? NECTAR_TURRET_VELOCITY_MODEL(TEST_DISTANCE_INCHES)
                    : DIRECT_TARGET_NECTAR_RPM;

            double currentPollenRPM = scoringSubsystem.getPollenTurretMotorRPM();
            double currentNectarRPM = scoringSubsystem.getNectarTurretMotorRPM();

            double pollenMotorPower = 0;
            double nectarMotorPower = 0;

            if (flywheelsEnabled) {
                if (pollenMotorPID.getTarget() != targetPollenRPM) pollenMotorPID.setTarget(targetPollenRPM);
                if (nectarMotorPID.getTarget() != targetNectarRPM) nectarMotorPID.setTarget(targetNectarRPM);

                pollenMotorPower = pollenMotorPID.calculateOutput(currentPollenRPM, dt);
//                Sanjay is the glorius one and the one above all and that is why i am the sucesser of abhikarthis bummy code
                nectarMotorPower = nectarMotorPID.calculateOutput(currentNectarRPM, dt);
            }

            scoringSubsystem.setPollenTurretMotorPower(pollenMotorPower);
            scoringSubsystem.setNectarTurretMotorPower(nectarMotorPower);

            panelsTelemetry.addData("Flywheels Enabled (Press A)", flywheelsEnabled);
            panelsTelemetry.addData("Gates Open / Firing (Hold B)", gamepad1.b);
            panelsTelemetry.addData("Pollen Target RPM", targetPollenRPM);
            panelsTelemetry.addData("Pollen Current RPM", currentPollenRPM);
            panelsTelemetry.addData("Pollen Motor Power", pollenMotorPower);
            panelsTelemetry.addData("Nectar Target RPM", targetNectarRPM);
            panelsTelemetry.addData("Nectar Current RPM", currentNectarRPM);
            panelsTelemetry.addData("Nectar Motor Power", nectarMotorPower);
            panelsTelemetry.update(telemetry);
        }
    }
}