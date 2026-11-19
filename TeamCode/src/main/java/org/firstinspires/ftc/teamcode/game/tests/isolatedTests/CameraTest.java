package org.firstinspires.ftc.teamcode.game.tests.isolatedTests;

import static org.firstinspires.ftc.teamcode.util.Constants.TEAM;

import android.util.Size;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.ftccommon.external.OnCreate;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name="Camera Test")
@Configurable
public class CameraTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        VisionPortal pollenTurretCamera;
        AprilTagProcessor aprilTag = new AprilTagProcessor.Builder().build();

        pollenTurretCamera = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "pollenTurretCamera"))
                .setCameraResolution(new Size(640, 400))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(aprilTag)
                .build();


        waitForStart();
        while (opModeIsActive()) {
            List<AprilTagDetection> detections = aprilTag.getDetections();
            double bearing = 361;
            for (AprilTagDetection detection : detections) {
                if (detection instanceof AprilTagClusterDetection) {

                    AprilTagClusterDetection clusterDet = (AprilTagClusterDetection) detection;

                    if (clusterDet.metadata != null) {

                        bearing = clusterDet.ftcPose.bearing;
                    }
                }
            }
            telemetry.addData("bearing", bearing);
            telemetry.update();
        }
    }
}
