package org.firstinspires.ftc.teamcode.game.tests.isolatedTests;

import static org.firstinspires.ftc.teamcode.util.Constants.ENCODER_TO_TURRET_RATIO;
import static org.firstinspires.ftc.teamcode.util.Constants.NECTAR_TURRET_SERVO_CONTROLLER;
import static org.firstinspires.ftc.teamcode.util.Constants.POLLEN_SERVO_KD;
import static org.firstinspires.ftc.teamcode.util.Constants.POLLEN_SERVO_KF;
import static org.firstinspires.ftc.teamcode.util.Constants.POLLEN_SERVO_KI;
import static org.firstinspires.ftc.teamcode.util.Constants.POLLEN_SERVO_KP;
import static org.firstinspires.ftc.teamcode.util.Constants.POLLEN_TURRET_SERVO_CONTROLLER;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.PIDController;

@Configurable
@TeleOp(name="TurretSetPositionTuner")
public class TurretSetPosition extends LinearOpMode {
    public static double pollenTargetPositionTurret = 0;
    @Override
    public void runOpMode() {
        CRServo pollenTurretServo = hardwareMap.get(CRServo.class, "pollenTurret"); // port 5 servo expansion hub
//        CRServo nectarTurretServo = hardwareMap.get(CRServo.class, "nectarTurret"); // port 5 servo control hub

        AnalogInput pollenTurretEncoder = hardwareMap.get(AnalogInput.class, "pollenEncoder"); // 0 analog control hub
//        AnalogInput nectarTurretEncoder = hardwareMap.get(AnalogInput.class, "nectarEncoder"); // 0 analog expansion hub

        PIDController pollenTurretServoController = new PIDController(POLLEN_TURRET_SERVO_CONTROLLER[0], POLLEN_TURRET_SERVO_CONTROLLER[1],
                POLLEN_TURRET_SERVO_CONTROLLER[2], POLLEN_TURRET_SERVO_CONTROLLER[3]);
//        PIDController nectarTurretServoController = new PIDController(NECTAR_TURRET_SERVO_CONTROLLER[0], NECTAR_TURRET_SERVO_CONTROLLER[1],
//                NECTAR_TURRET_SERVO_CONTROLLER[2], NECTAR_TURRET_SERVO_CONTROLLER[3]);
        ElapsedTime timer = new ElapsedTime();

        waitForStart();
        timer.reset();
        while (opModeIsActive()) {
//            double nectarTargetPositionTurret = gamepad1.left_stick_x * 180;
            ;

//            double nectarTargetPositionEncoder = nectarTargetPositionTurret / ENCODER_TO_TURRET_RATIO;
            double pollenTargetPositionEncoder = pollenTargetPositionTurret / ENCODER_TO_TURRET_RATIO;

            // Read the individual constants (not the array) so Panels edits apply live.
            pollenTurretServoController.setConstants(POLLEN_SERVO_KP, POLLEN_SERVO_KI, POLLEN_SERVO_KD, POLLEN_SERVO_KF);
//            nectarTurretServoController.setConstants(NECTAR_TURRET_SERVO_CONTROLLER[0], NECTAR_TURRET_SERVO_CONTROLLER[1],
//                    NECTAR_TURRET_SERVO_CONTROLLER[2], NECTAR_TURRET_SERVO_CONTROLLER[3]);

            // setTarget() resets the integral, so only call it when the target changes.
            if (pollenTurretServoController.getTarget() != pollenTargetPositionTurret) {
                pollenTurretServoController.setTarget(pollenTargetPositionTurret);
            }
//            nectarTurretServoController.setTarget(nectarTargetPositionTurret);
            double angle = getPollenTurretAngleDegrees(pollenTurretEncoder);
            double rawAngle =getPollenTurretAngleDegreesRaw(pollenTurretEncoder);
            double output = pollenTurretServoController.calculateOutput(angle, timer.seconds());

            pollenTurretServo.setPower(output);

//            telemetry.addData("Nectar Target Position", nectarTargetPositionTurret);
            telemetry.addData("Pollen Target Position", pollenTargetPositionTurret);
            telemetry.addData("Turret Power", output);
            telemetry.addData("curr Pos", angle);
            telemetry.addData("raw angle", rawAngle);

            telemetry.update();

            timer.reset();
        }
    }

    public double getPollenTurretAngleDegrees(AnalogInput pollenTurretEncoder) {
        double voltage = pollenTurretEncoder.getVoltage();
        double rawDegrees = (voltage / 3.3) * 360.0;
        rawDegrees -= 180;
        rawDegrees = -rawDegrees * ENCODER_TO_TURRET_RATIO;



        return rawDegrees;
    }

    public double getPollenTurretAngleDegreesRaw(AnalogInput pollenTurretEncoder) {
        double voltage = pollenTurretEncoder.getVoltage();
        double rawDegrees = (voltage / 3.3) * 360.0;

        // Force the wrap-around to become negative degrees
        // Turning backwards past 0 (e.g., 350 degrees) becomes -10 degrees

//        if (rawDegrees > 180.0) {
//            rawDegrees -= 360.0;
//        }

//        rawDegrees *= -1;

        return rawDegrees;
    }

    public double getNectarTurretAngleDegrees(AnalogInput nectarTurretEncoder) {
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
}
