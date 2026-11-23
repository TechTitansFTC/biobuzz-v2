package org.firstinspires.ftc.teamcode.game.tests.isolatedTests;

import static org.firstinspires.ftc.teamcode.util.Constants.INTAKE_MOTOR_POWER;
import static org.firstinspires.ftc.teamcode.util.Constants.POLLEN_GATE_CLOSE_POSITION;
import static org.firstinspires.ftc.teamcode.util.Constants.POLLEN_GATE_OPEN_POSITION;
import static org.firstinspires.ftc.teamcode.util.Constants.TEAM;
import static org.firstinspires.ftc.teamcode.util.Constants.TRANSFER_MOTOR_POWER;

import android.util.Size;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.teamcode.util.PanelsFrameSource;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * Custom velocity PIDF for the pollen shooter.
 *
 *   power = [ kS + kV * target ] * (12V / battery)      <- feedforward: holds speed
 *         + kP * error + kI * integral(error) + kD * d(error)/dt   <- feedback: fixes dips
 *
 * Controls:
 *   A          - toggle shooter on/off at TARGET_RPM
 *   Dpad up/dn - TARGET_RPM +/- RPM_STEP
 *   B (hold)   - full power open loop; watch "Max RPM" to get kV
 * All gains are editable live from Panels. Graph "Target RPM" vs "Current RPM".
 */
@TeleOp(name="shootingTest")
@Configurable
public class ShootingTest extends LinearOpMode {
    public static double TARGET_RPM = 3100;
    public static double RPM_STEP = 100;

    public static double kS = 0.0;
    public static double kV = 0.00017;   // ~1 / (max RPM at 12V)
    public static double kP = 0.001;
    public static double kI = 0.000007;
    public static double kD = 0.0;

    public static double TICKS_PER_REV = 28;      // bare motor encoder
    public static double NOMINAL_VOLTAGE = 12.0;
    public static boolean REVERSE_MOTOR = true;    // flips which way the wheel spins
    public static boolean REVERSE_ENCODER = true;  // flips only the RPM reading (set true if RPM reads negative)
    public static double RPM_FILTER = 0.3;  // 1 = no smoothing, lower = smoother
    public static boolean FEED = true;   // intake + transfer on, pollen gate open

    @Override
    public void runOpMode() {
        TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();
        VisionPortal pollenTurretCamera;
        AprilTagProcessor aprilTag = new AprilTagProcessor.Builder().setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES).build();
        PanelsFrameSource panelsFrames = new PanelsFrameSource();
        pollenTurretCamera = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "pollenTurretCamera"))
                .setCameraResolution(new Size(1280, 800))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(aprilTag)
                .addProcessor(panelsFrames)
                .build();

        DcMotorEx pollenShooter = hardwareMap.get(DcMotorEx.class, "pollenShooter");
        pollenShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pollenShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // we do our own control
        pollenShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Intake + transfer run the whole time so pollen can be fed straight through to the shooter.
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);    // same as IntakeSubsystem
        DcMotor transfer = hardwareMap.get(DcMotor.class, "transfer");
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);  // same as StorageSubsystem
        Servo pollenGate = hardwareMap.get(Servo.class, "pollenGate");

        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();
        ElapsedTime loopTimer = new ElapsedTime();

        PIDController pid = new PIDController(kP, kI, kD, 0);

        boolean enabled = true;
        boolean aLast = false, upLast = false, downLast = false;
        double maxRpm = 0;
        double filteredRpm = 0;
        double distance = 36;

        waitForStart();
        loopTimer.reset();

        while (opModeIsActive()) {
            pollenShooter.setDirection(REVERSE_MOTOR ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

            if (gamepad1.a && !aLast) enabled = !enabled;
            if (gamepad1.dpad_up && !upLast) TARGET_RPM += RPM_STEP;
            if (gamepad1.dpad_down && !downLast) TARGET_RPM = Math.max(0, TARGET_RPM - RPM_STEP);
            aLast = gamepad1.a;
            upLast = gamepad1.dpad_up;
            downLast = gamepad1.dpad_down;

            double dt = loopTimer.seconds();
            loopTimer.reset();
            if (dt <= 0 || dt > 0.5) dt = 0.02;

            double voltage = battery.getVoltage();
            if (voltage < 1) voltage = NOMINAL_VOLTAGE;

            // Raw encoder velocity is noisy; smooth it before the PID sees it.
            // RPM_FILTER = 1 -> no filtering, smaller -> smoother but slower to react.
            // setDirection() flips power AND encoder together, so if the encoder counts the
            // opposite way from the power, only this flag can fix it.
            double rawRpm = pollenShooter.getVelocity() * 60.0 / TICKS_PER_REV;
            if (REVERSE_ENCODER) rawRpm = -rawRpm;
            filteredRpm += Range.clip(RPM_FILTER, 0.01, 1) * (rawRpm - filteredRpm);
            double rpm = filteredRpm;

            List<AprilTagDetection> detections = aprilTag.getDetections();

            for (AprilTagDetection detection : detections) {
                if (detection instanceof AprilTagClusterDetection) {

                    AprilTagClusterDetection clusterDet = (AprilTagClusterDetection) detection;

                    if (clusterDet.metadata != null) {

                        if (!clusterDet.metadata.name.startsWith(TEAM)) {
                            continue;
                        }

                        distance = clusterDet.ftcPose.range;
                    }
                }
            }

            telemetry.addData("Distance", distance);
            panels.addData("Distance", distance);

            double target = TARGET_RPM;
            double error = target - rpm;

            // Gains are re-read every loop so Panels edits apply live.
            // kF is 0 here: PIDController's kF is a constant, but a flywheel needs feedforward
            // proportional to the target, so that's done separately below with kS/kV.
            pid.setConstants(kP, kI, kD, 0);

            // setTarget() resets the integral, so only call it when the target actually changes.
            target = TARGET_RPM;
            if (target != pid.getTarget()) {
                pid.setTarget(target);
            }

            double power;
            if (gamepad1.b) {
                power = 1.0;
                maxRpm = Math.max(maxRpm, rpm);
            } else if (target <= 0) {
                power = 0;
            } else {
                // Feedforward: power needed to hold the target speed, scaled up as the battery drops.
                double feedforward = (kS + kV * target) * (NOMINAL_VOLTAGE / voltage);
                // Feedback: PID corrects whatever error is left (e.g. the dip after a shot).
                double feedback = pid.calculateOutput(rpm, dt);

                power = Range.clip(feedforward + feedback, 0, 1); // never drive the flywheel backwards
            }

            pollenShooter.setPower(power);

            intake.setPower(FEED ? INTAKE_MOTOR_POWER : 0);
            transfer.setPower(FEED ? TRANSFER_MOTOR_POWER : 0);
            pollenGate.setPosition(FEED ? POLLEN_GATE_OPEN_POSITION : POLLEN_GATE_CLOSE_POSITION);

            panels.addData("Feeding (intake/transfer/gate)", FEED);
            panels.addData("Target RPM", target);
            panels.addData("Current RPM", rpm);
            panels.addData("Raw RPM", rawRpm);
            panels.addData("Error RPM", error);
            panels.addData("Power", power);
            panels.addData("Battery V", voltage);
            panels.addData("Max RPM (hold B)", maxRpm);
            panels.addData("Suggested kV", maxRpm > 0 ? (voltage / NOMINAL_VOLTAGE) / maxRpm : 0);
            panels.addData("Loop ms", dt * 1000);
            panels.update(telemetry);
        }

        pollenShooter.setPower(0);
        intake.setPower(0);
        transfer.setPower(0);
    }
}
