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
import org.firstinspires.ftc.teamcode.util.PIDController;

@Configurable
@TeleOp(name = "2. Turret Servo & Vision Tuner", group = "Tuners")
public class TurretServoTunerOpMode extends LinearOpMode {
    // 0 = Manual Joystick, 1 = Angle PID Tuner, 2 = AprilTag Tracking
    public static int TUNING_MODE = 1;
    public static double TARGET_POLLEN_ANGLE = 0.0;
    public static double TARGET_NECTAR_ANGLE = 0.0;
    public static double TEST_LATERAL_DISTANCE = 48.0;

    @Override
    public void runOpMode() {
        CommandScheduler.getInstance().reset();
        TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        ScoringSubsystem scoringSubsystem = new ScoringSubsystem(hardwareMap, telemetry);

        PIDController pollenServoPID = new PIDController(POLLEN_SERVO_KP, POLLEN_SERVO_KI, POLLEN_SERVO_KD, POLLEN_SERVO_KF);
        PIDController nectarServoPID = new PIDController(NECTAR_SERVO_KP, NECTAR_SERVO_KI, NECTAR_SERVO_KD, NECTAR_SERVO_KF);

        ElapsedTime loopTimer = new ElapsedTime();
        boolean dpadLeftLast = false, dpadRightLast = false;

        waitForStart();
        loopTimer.reset();

        while (opModeIsActive()) {
            double dt = loopTimer.seconds();
            loopTimer.reset();

            // Update PID constants live from Panels
            pollenServoPID.setConstants(POLLEN_SERVO_KP, POLLEN_SERVO_KI, POLLEN_SERVO_KD, POLLEN_SERVO_KF);
            nectarServoPID.setConstants(NECTAR_SERVO_KP, NECTAR_SERVO_KI, NECTAR_SERVO_KD, NECTAR_SERVO_KF);

            // D-Pad Left/Right bumps target angles by 15 degrees for quick PID step testing
            if (gamepad1.dpad_right && !dpadRightLast) {
                TARGET_POLLEN_ANGLE += 15.0;
                TARGET_NECTAR_ANGLE += 15.0;
            }
            if (gamepad1.dpad_left && !dpadLeftLast) {
                TARGET_POLLEN_ANGLE -= 15.0;
                TARGET_NECTAR_ANGLE -= 15.0;
            }
            dpadRightLast = gamepad1.dpad_right;
            dpadLeftLast = gamepad1.dpad_left;

            double pollenAngle = scoringSubsystem.getPollenTurretAngleDegrees();
            double nectarAngle = scoringSubsystem.getNectarTurretAngleDegrees();
            double bearing = scoringSubsystem.getBearing();

            double pollenPower = 0;
            double nectarPower = 0;

            if (TUNING_MODE == 0) {
                // Mode 0: Manual CRServo control via joysticks
                pollenPower = gamepad1.left_stick_x;
                nectarPower = gamepad1.right_stick_x;
            } else if (TUNING_MODE == 1) {
                // Mode 1: Direct Angle PID Tuning
                if (pollenServoPID.getTarget() != TARGET_POLLEN_ANGLE) pollenServoPID.setTarget(TARGET_POLLEN_ANGLE);
                if (nectarServoPID.getTarget() != TARGET_NECTAR_ANGLE) nectarServoPID.setTarget(TARGET_NECTAR_ANGLE);

                pollenPower = pollenServoPID.calculateOutput(pollenAngle, dt);
                nectarPower = nectarServoPID.calculateOutput(nectarAngle, dt);
            } else if (TUNING_MODE == 2) {
                // Mode 2: Live AprilTag Alignment Test
                double nectarTarget = getSecondTurretBearing(TEST_LATERAL_DISTANCE, TURRET_OFFSET_DISTANCE, bearing);
                pollenServoPID.setTarget(0.0);
                nectarServoPID.setTarget(nectarTarget);

                pollenPower = pollenServoPID.calculateOutput(bearing, dt);
                nectarPower = nectarServoPID.calculateOutput(nectarAngle, dt);
            }

            scoringSubsystem.setPollenTurretServoPower(pollenPower);
            scoringSubsystem.setNectarTurretServoPower(nectarPower);

            panelsTelemetry.addData("Tuning Mode (0=Manual, 1=PID, 2=Vision)", TUNING_MODE);
            panelsTelemetry.addData("Pollen Target Angle", pollenServoPID.getTarget());
            panelsTelemetry.addData("Pollen Current Angle", pollenAngle);
            panelsTelemetry.addData("Pollen Servo Power", pollenPower);
            panelsTelemetry.addData("Nectar Target Angle", nectarServoPID.getTarget());
            panelsTelemetry.addData("Nectar Current Angle", nectarAngle);
            panelsTelemetry.addData("Nectar Servo Power", nectarPower);
            panelsTelemetry.addData("AprilTag Bearing", bearing);
            panelsTelemetry.update(telemetry);
        }
    }

    private double getSecondTurretBearing(double x, double y, double thetaDegrees) {
        double theta = Math.toRadians(thetaDegrees);
        double xComponent = x * Math.sin(theta);
        double yComponent = y - (x * Math.cos(theta));
        return Math.toDegrees(Math.atan2(xComponent, yComponent));
    }
}