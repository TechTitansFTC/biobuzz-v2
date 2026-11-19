package org.firstinspires.ftc.teamcode.game.tests.isolatedTests;

import static org.firstinspires.ftc.teamcode.util.Constants.TEAM;

import android.util.Size;

import com.bylazar.camerastream.PanelsCameraStream;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.ftccommon.external.OnCreate;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.PanelsFrameSource;
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
        PanelsFrameSource panelsFrames = new PanelsFrameSource();

        pollenTurretCamera = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "pollenTurretCamera"))
                .setCameraResolution(new Size(1280, 800))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(aprilTag)
                .addProcessor(panelsFrames)
                .build();

        PanelsCameraStream.INSTANCE.startStream(panelsFrames, 15);

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

            telemetry.addData("Camera", pollenTurretCamera.getCameraState());
            telemetry.addData("Detections", detections.size());
            telemetry.addData("bearing", bearing);
            telemetry.update();
        }

        PanelsCameraStream.INSTANCE.stopStream();
        pollenTurretCamera.close();

    }
}
