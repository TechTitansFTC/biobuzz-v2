package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.util.Constants.TEAM;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class ScoringSubsystem extends SubsystemBase {
    private final DcMotor pollenTurretMotor;
    private final DcMotor nectarTurretMotor;
    private final CRServo pollenTurretServo;
    private final CRServo nectarTurretServo;

    // The processor actually calculates and stores the AprilTag data
    private final AprilTagProcessor aprilTag;
    private final VisionPortal pollenTurretCamera;

    private final Telemetry telemetry;

    public ScoringSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        pollenTurretMotor = hardwareMap.get(DcMotor.class, "pollenTurretMotor");
        nectarTurretMotor = hardwareMap.get(DcMotor.class, "nectarTurretMotor");
        pollenTurretServo = hardwareMap.get(CRServo.class, "pollenTurretServo");
        nectarTurretServo = hardwareMap.get(CRServo.class, "nectarTurretServo");

        // 1. Initialize the AprilTag processor
        aprilTag = new AprilTagProcessor.Builder().build();

        // 2. Build the VisionPortal and attach the processor to the physical webcam
        pollenTurretCamera = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "pollenTurretCamera"))
                .addProcessor(aprilTag)
                .build();
    }

    @Override
    public void periodic() {
        // No parameters needed anymore
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

    // Removed the parameter from the method signature
    public Double getBearing() {
        // Retrieve the detections directly from the processor instance
        List<AprilTagDetection> detections = aprilTag.getDetections();

        for (AprilTagDetection detection : detections) {

            // Check if it is a BIOBUZZ Cluster instead of a Single Tag
            if (detection instanceof AprilTagClusterDetection) {

                // Cast the detection to a Cluster so we can read its data
                AprilTagClusterDetection clusterDet = (AprilTagClusterDetection) detection;

                if (clusterDet.metadata != null) {

                    // Optional: If you only wanted RED alliance targets,
                    if (!clusterDet.metadata.name.startsWith(TEAM)) {
                        continue;
                    }

                    // The SDK has already merged the tags. Return the bearing to the center of the CELL!
                    return clusterDet.ftcPose.bearing;
                }
            }
        }

        // Return null if no clusters are currently in view
        return null;
    }
}