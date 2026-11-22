package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.game.tests.isolatedTests.ShootingTest.TARGET_RPM;
import static org.firstinspires.ftc.teamcode.util.Constants.ENCODER_TO_TURRET_RATIO;
import static org.firstinspires.ftc.teamcode.util.Constants.NOMINAL_VOLTAGE;
import static org.firstinspires.ftc.teamcode.util.Constants.POLLEN_SHOOTER_ENCODER_REVERSED;
import static org.firstinspires.ftc.teamcode.util.Constants.POLLEN_SHOOTER_MOTOR_REVERSED;
import static org.firstinspires.ftc.teamcode.util.Constants.POLLEN_SHOOTER_KD;
import static org.firstinspires.ftc.teamcode.util.Constants.POLLEN_SHOOTER_KI;
import static org.firstinspires.ftc.teamcode.util.Constants.POLLEN_SHOOTER_KP;
import static org.firstinspires.ftc.teamcode.util.Constants.POLLEN_SHOOTER_KS;
import static org.firstinspires.ftc.teamcode.util.Constants.POLLEN_SHOOTER_KV;
import static org.firstinspires.ftc.teamcode.util.Constants.POLLEN_SHOOTER_RPM_FILTER;
import static org.firstinspires.ftc.teamcode.util.Constants.POLLEN_SHOOTER_RPM_TOLERANCE;
import static org.firstinspires.ftc.teamcode.util.Constants.POLLEN_SHOOTER_TARGET_RPM;
import static org.firstinspires.ftc.teamcode.util.Constants.TEAM;

import android.util.Size;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class ScoringSubsystem extends SubsystemBase {
    private final DcMotorEx pollenTurretMotor;
    private final DcMotorEx nectarTurretMotor;
    private final CRServo pollenTurretServo;
    private final CRServo nectarTurretServo;
    private final AnalogInput pollenTurretEncoder;
    private final AnalogInput nectarTurretEncoder;
    private final AprilTagProcessor aprilTag;
    private final VisionPortal pollenTurretCamera;
    private final VoltageSensor battery;
    private final Telemetry telemetry;
    private final Follower follower;
    private double distanceToScore;

    // Pollen flywheel velocity control (runs in periodic)
    private final PIDController shooterController =
            new PIDController(POLLEN_SHOOTER_KP, POLLEN_SHOOTER_KI, POLLEN_SHOOTER_KD, 0);
    private final ElapsedTime shooterLoopTimer = new ElapsedTime();
    private double filteredRpm = 0;
    private boolean firstShooterUpdate = true;
    private boolean pollenShooterEnabled = true; // on for the whole match unless toggled off with B
    private static double POLLEN_SHOOTER_TARGET_RPM_CHANGE = POLLEN_SHOOTER_TARGET_RPM;

    public ScoringSubsystem(HardwareMap hardwareMap, Telemetry telemetry, Follower follower) {
        this.telemetry = telemetry;

        pollenTurretMotor = hardwareMap.get(DcMotorEx.class, "pollenShooter"); // port 3 motor control hub
        nectarTurretMotor = hardwareMap.get(DcMotorEx.class, "nectarShooter"); // port 0 motor expansion hub (encoder)
        pollenTurretServo = hardwareMap.get(CRServo.class, "pollenTurret"); // port 5 servo expansion hub
        nectarTurretServo = hardwareMap.get(CRServo.class, "nectarTurret"); // port 5 servo control hub

        pollenTurretEncoder = hardwareMap.get(AnalogInput.class, "pollenEncoder"); // 0 analog control hub
        nectarTurretEncoder = hardwareMap.get(AnalogInput.class, "nectarEncoder"); // 0 analog expansion hub

        pollenTurretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        nectarTurretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Let the flywheel coast down instead of braking it hard (same as ShootingTest).
        pollenTurretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        battery = hardwareMap.voltageSensor.iterator().next();
        this.follower = follower;
        distanceToScore = getDistanceToScore(follower);

        aprilTag = new AprilTagProcessor.Builder().setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES).build();

        // Same camera settings as CameraTest, where tag detection works.
        pollenTurretCamera = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "pollenTurretCamera"))
                .setCameraResolution(new Size(1280, 800))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(aprilTag)
                .build();

        register();
    }

    @Override
    public void periodic() {
        updatePollenShooter();
        distanceToScore = getDistanceToScore(follower);

        telemetry.addData("Camera bearing", getBearing());
        telemetry.addData("Pollen shooter on (B)", pollenShooterEnabled);
        telemetry.addData("Pollen shooter RPM", filteredRpm);
        telemetry.addData("Pollen shooter target", POLLEN_SHOOTER_TARGET_RPM);
        telemetry.addData("Pollen shooter at speed", isPollenShooterAtSpeed());
        telemetry.addData("", "");
        telemetry.addData("NOW DATA", "");
        telemetry.addData("Distance to score", distanceToScore);
        telemetry.addData("pose x", follower.pose().x());
        telemetry.addData("pose y", follower.pose().y());
    }

    /**
     * Holds the pollen flywheel at POLLEN_SHOOTER_TARGET_RPM. Runs every loop from periodic(),
     * so the flywheel stays on for the whole OpMode no matter which command is running.
     * Voltage-compensated feedforward + PID, same as ShootingTest.
     */
    private void updatePollenShooter() {
        double dt = shooterLoopTimer.seconds();
        shooterLoopTimer.reset();

        pollenTurretMotor.setDirection(POLLEN_SHOOTER_MOTOR_REVERSED
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        double rawRpm = getPollenTurretMotorRPM();
        if (firstShooterUpdate) {
            filteredRpm = rawRpm;
            dt = 0.02;
            firstShooterUpdate = false;
        }
        filteredRpm += Range.clip(POLLEN_SHOOTER_RPM_FILTER, 0.01, 1) * (rawRpm - filteredRpm);

        double distance = getDistance();
        // Re-read gains every loop so Panels edits apply live.
        shooterController.setConstants(POLLEN_SHOOTER_KP, POLLEN_SHOOTER_KI, POLLEN_SHOOTER_KD, 0);
        double target = TARGET_RPM;
        if (shooterController.getTarget() != target) {
            shooterController.setTarget(target);
        }

        if (!pollenShooterEnabled) {
            pollenTurretMotor.setPower(0); // toggled off with B: coast down
            return;
        }

        double voltage = battery.getVoltage();
        if (voltage < 1) voltage = NOMINAL_VOLTAGE;

        double feedforward = (POLLEN_SHOOTER_KS + POLLEN_SHOOTER_KV * POLLEN_SHOOTER_TARGET_RPM) * (NOMINAL_VOLTAGE / voltage);
        double feedback = shooterController.calculateOutput(filteredRpm, dt);

        pollenTurretMotor.setPower(Range.clip(feedforward + feedback, 0, 1)); // never drive the flywheel backwards
    }

    /** Turns the always-on flywheel off or back on (bound to B in TeleOp). On by default. */
    public void togglePollenShooter() {
        pollenShooterEnabled = !pollenShooterEnabled;
        if (pollenShooterEnabled) {
            // Start the PID fresh so stale integral/derivative from before doesn't carry over.
            shooterController.setTarget(POLLEN_SHOOTER_TARGET_RPM);
        }
    }

    public boolean isPollenShooterEnabled() {
        return pollenShooterEnabled;
    }

    /** Smoothed flywheel RPM (what the shooter PID uses). */
    public double getPollenShooterRpm() {
        return filteredRpm;
    }

    public boolean isPollenShooterAtSpeed() {
        return pollenShooterEnabled && Math.abs(POLLEN_SHOOTER_TARGET_RPM - filteredRpm) <= POLLEN_SHOOTER_RPM_TOLERANCE;
    }

    public void setPollenTurretMotorPower(double power) {
        pollenTurretMotor.setPower(power);
    }

    public void setNectarTurretMotorPower(double power) {
        nectarTurretMotor.setPower(power);
    }

    public void setPollenTurretServoPower(double power) {
        pollenTurretServo.setPower(power);
    }

    public void setNectarTurretServoPower(double power) {
        nectarTurretServo.setPower(power);
    }

    public double getPollenTurretMotorRPM() {
        double ticksPerSecond = pollenTurretMotor.getVelocity();
        double rpm = (ticksPerSecond * 60) / 28.0;

        // setDirection() flips power and encoder together; this flips only the reading.
        return POLLEN_SHOOTER_ENCODER_REVERSED ? -rpm : rpm;
    }

    public double getBatteryVoltage() {
        return battery.getVoltage();
    }

    public double getNectarTurretMotorRPM() {
        double ticksPerSecond = nectarTurretMotor.getVelocity();

        return (ticksPerSecond * 60) / 28.0;
    }

    public Double getBearing() {
        List<AprilTagDetection> detections = aprilTag.getDetections();

        for (AprilTagDetection detection : detections) {
            if (detection instanceof AprilTagClusterDetection) {

                AprilTagClusterDetection clusterDet = (AprilTagClusterDetection) detection;

                if (clusterDet.metadata != null) {

                    if (!clusterDet.metadata.name.startsWith(TEAM)) {
                        continue;
                    }

                    return -clusterDet.ftcPose.bearing;
                }
            }
        }

        return 361.0;
    }

    public Double getDistance() {
        List<AprilTagDetection> detections = aprilTag.getDetections();

        for (AprilTagDetection detection : detections) {
            if (detection instanceof AprilTagClusterDetection) {

                AprilTagClusterDetection clusterDet = (AprilTagClusterDetection) detection;

                if (clusterDet.metadata != null) {

                    if (!clusterDet.metadata.name.startsWith(TEAM)) {
                        continue;
                    }

                    return clusterDet.ftcPose.range;
                }
            }
        }

        return -1.0;
    }

    public double getPollenTurretAngleDegrees() {
        double voltage = pollenTurretEncoder.getVoltage();
        double rawDegrees = (voltage / 3.3) * 360.0;

        // Force the wrap-around to become negative degrees
        // Turning backwards past 0 (e.g., 350 degrees) becomes -10 degrees
        if (rawDegrees > 180.0) {
            rawDegrees -= 360.0;
        }

        return rawDegrees * ENCODER_TO_TURRET_RATIO;
    }

    public double getNectarTurretAngleDegrees() {
        double voltage = nectarTurretEncoder.getVoltage();
        double maxVoltage = 3.3;

        double rawDegrees = (voltage / 3.3) * 360.0;

        // Force the wrap-around to become negative degrees
        // Turning backwards past 0 (e.g., 350 degrees) becomes -10 degrees
        if (rawDegrees > 180.0) {
            rawDegrees -= 360.0;
        }

        return rawDegrees * ENCODER_TO_TURRET_RATIO;
    }

    public double getDistanceToScore(Follower follower) {
        double robotX = follower.pose().x();
        double robotY = follower.pose().y();
        double score1X;
        double score2X;
        if (TEAM.equals("R")) {
            score1X = 57;
            score2X = 57;
        } else {
            score1X = 81;
            score2X = 81;
        }

        double score1Y = 48;
        double score2Y = 96;

        double distanceFromScore1 = Math.sqrt(Math.pow(robotX - score1X, 2) + Math.pow(robotY - score1Y, 2));
        double distanceFromScore2 = Math.sqrt(Math.pow(robotX - score2X, 2) + Math.pow(robotY - score2Y, 2));

        return Math.min(distanceFromScore1, distanceFromScore2);
    }
}