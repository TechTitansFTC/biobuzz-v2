package org.firstinspires.ftc.teamcode.game.tests.isolatedTests;

import static org.firstinspires.ftc.teamcode.game.tests.isolatedTests.TurretSetPosition.pollenTargetPositionTurret;
import static org.firstinspires.ftc.teamcode.util.Constants.ENCODER_TO_TURRET_RATIO;
import static org.firstinspires.ftc.teamcode.util.Constants.POLLEN_TURRET_SERVO_CONTROLLER;
import static org.firstinspires.ftc.teamcode.util.Constants.TEAM;

import android.util.Size;

import com.bylazar.camerastream.PanelsCameraStream;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.teamcode.util.PanelsFrameSource;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name="Camera Test")
@Configurable
public class CameraTest extends LinearOpMode {
    public static double kp = 0.003;
    public static double ki = 0;
    public static double kd = 0.000000004;
    public static double kf = 0.06;
    public static boolean reverse = false;
    @Override
    public void runOpMode() {
        VisionPortal pollenTurretCamera;
        AprilTagProcessor aprilTag = new AprilTagProcessor.Builder().build();
        CRServo pollenTurretServo = hardwareMap.get(CRServo.class, "pollenTurret"); // port 5 servo expansion hub

        PanelsFrameSource panelsFrames = new PanelsFrameSource();

        PIDController pollenTurretServoController = new PIDController(kp, ki,
                kd, kf);

        pollenTurretCamera = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "pollenTurretCamera"))
                .setCameraResolution(new Size(1280, 800))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(aprilTag)
                .addProcessor(panelsFrames)
                .build();

        PanelsCameraStream.INSTANCE.startStream(panelsFrames, 15);
        ElapsedTime timer = new ElapsedTime();

        waitForStart();
        timer.reset();
        while (opModeIsActive()) {
            List<AprilTagDetection> detections = aprilTag.getDetections();
            double bearing = 0;
            for (AprilTagDetection detection : detections) {
                if (detection instanceof AprilTagClusterDetection) {

                    AprilTagClusterDetection clusterDet = (AprilTagClusterDetection) detection;

                    if (clusterDet.metadata != null) {

                        bearing = -clusterDet.ftcPose.bearing;
                    }
                }
            }

            pollenTurretServoController.setConstants(kp, ki,
                    kd, kf);
//            nectarTurretServoController.setConstants(NECTAR_TURRET_SERVO_CONTROLLER[0], NECTAR_TURRET_SERVO_CONTROLLER[1],
//                    NECTAR_TURRET_SERVO_CONTROLLER[2], NECTAR_TURRET_SERVO_CONTROLLER[3]);

            pollenTurretServoController.setTarget(0);
//            nectarTurretServoController.setTarget(nectarTargetPositionTurret);
            double angle = -bearing;
            double output = pollenTurretServoController.calculateOutput(angle, timer.seconds());
            if (reverse) output = - output;
            pollenTurretServo.setPower(output);


//            telemetry.addData("Nectar Target Position", nectarTargetPositionTurret);
            telemetry.addData("Turret Power", output);

            timer.reset();
            telemetry.addData("Camera", pollenTurretCamera.getCameraState());
            telemetry.addData("Detections", detections.size());
            telemetry.addData("bearing", bearing);
            telemetry.update();
        }

        PanelsCameraStream.INSTANCE.stopStream();
        pollenTurretCamera.close();

    }
}