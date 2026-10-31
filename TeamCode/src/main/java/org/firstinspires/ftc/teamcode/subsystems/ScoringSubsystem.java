package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.util.Constants.TEAM;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
    private final Servo pollenGateServo;
    private final Servo nectarGateServo;
    private final AnalogInput pollenTurretEncoder;
    private final AnalogInput nectarTurretEncoder;
    private final AprilTagProcessor aprilTag;
    private final VisionPortal pollenTurretCamera;
    private final Telemetry telemetry;

    public ScoringSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        pollenTurretMotor = hardwareMap.get(DcMotorEx.class, "pollenTurretMotor");
        nectarTurretMotor = hardwareMap.get(DcMotorEx.class, "nectarTurretMotor");
        pollenTurretServo = hardwareMap.get(CRServo.class, "pollenTurretServo");
        nectarTurretServo = hardwareMap.get(CRServo.class, "nectarTurretServo");
        pollenGateServo = hardwareMap.get(Servo.class, "pollenGateServo");
        nectarGateServo = hardwareMap.get(Servo.class, "nectarGateServo");
        pollenTurretEncoder = hardwareMap.get(AnalogInput.class, "pollenTurretEncoder");
        nectarTurretEncoder = hardwareMap.get(AnalogInput.class, "nectarTurretEncoder");

        pollenTurretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        nectarTurretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        aprilTag = new AprilTagProcessor.Builder().build();

        pollenTurretCamera = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "pollenTurretCamera"))
                .addProcessor(aprilTag)
                .build();
    }

    @Override
    public void periodic() {
        telemetry.addData("Camera bearing", getBearing());
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

        return (ticksPerSecond * 60) / 28.0;
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

                    return clusterDet.ftcPose.bearing;
                }
            }
        }
        return null;
    }

    public double getPollenTurretAngleDegrees() {
        double voltage = pollenTurretEncoder.getVoltage();
        double maxVoltage = 3.3;

        return (voltage / maxVoltage) * 360.0;
    }

    public double getNectarTurretAngleDegrees() {
        double voltage = nectarTurretEncoder.getVoltage();
        double maxVoltage = 3.3;

        return (voltage / maxVoltage) * 360.0;
    }

    public void setPollenGateServoPosition(double position) {
        pollenGateServo.setPosition(position);
    }

    public void setNectarGateServoPosition(double position) {
        nectarGateServo.setPosition(position);
    }
}