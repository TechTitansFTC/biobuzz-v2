package org.firstinspires.ftc.teamcode.game.tests.isolatedTests;

import static org.firstinspires.ftc.teamcode.util.Constants.ENCODER_TO_TURRET_RATIO;
import static org.firstinspires.ftc.teamcode.util.Constants.NECTAR_TURRET_SERVO_CONTROLLER;
import static org.firstinspires.ftc.teamcode.util.Constants.POLLEN_TURRET_SERVO_CONTROLLER;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.util.PIDController;

@TeleOp(name="TurretSetPositionTuner")
public class TurretSetPosition extends LinearOpMode {
    @Override
    public void runOpMode() {
        CRServo pollenTurretServo = hardwareMap.get(CRServo.class, "pollenTurret"); // port 5 servo expansion hub
        CRServo nectarTurretServo = hardwareMap.get(CRServo.class, "nectarTurret"); // port 5 servo control hub

        AnalogInput pollenTurretEncoder = hardwareMap.get(AnalogInput.class, "pollenEncoder"); // 0 analog control hub
        AnalogInput nectarTurretEncoder = hardwareMap.get(AnalogInput.class, "nectarEncoder"); // 0 analog expansion hub

        PIDController pollenTurretServoController = new PIDController(POLLEN_TURRET_SERVO_CONTROLLER[0], POLLEN_TURRET_SERVO_CONTROLLER[1],
                POLLEN_TURRET_SERVO_CONTROLLER[2], POLLEN_TURRET_SERVO_CONTROLLER[3]);
        PIDController nectarTurretServoController = new PIDController(NECTAR_TURRET_SERVO_CONTROLLER[0], NECTAR_TURRET_SERVO_CONTROLLER[1],
                NECTAR_TURRET_SERVO_CONTROLLER[2], NECTAR_TURRET_SERVO_CONTROLLER[3]);

        waitForStart();
        while (opModeIsActive()) {
            double nectarTargetPositionTurret = gamepad1.left_stick_x * 180;
            double pollenTargetPositionTurret = gamepad1.right_stick_x * 180;

            double nectarTargetPositionEncoder = nectarTargetPositionTurret / ENCODER_TO_TURRET_RATIO;
            double pollenTargetPositionEncoder = pollenTargetPositionTurret / ENCODER_TO_TURRET_RATIO;

            pollenTurretServoController.setConstants(POLLEN_TURRET_SERVO_CONTROLLER[0], POLLEN_TURRET_SERVO_CONTROLLER[1],
                    POLLEN_TURRET_SERVO_CONTROLLER[2], POLLEN_TURRET_SERVO_CONTROLLER[3]);
            nectarTurretServoController.setConstants(NECTAR_TURRET_SERVO_CONTROLLER[0], NECTAR_TURRET_SERVO_CONTROLLER[1],
                    NECTAR_TURRET_SERVO_CONTROLLER[2], NECTAR_TURRET_SERVO_CONTROLLER[3]);

            pollenTurretServoController.setTarget(pollenTargetPositionTurret);
            nectarTurretServoController.setTarget(nectarTargetPositionTurret);

//            pollenTurretServoController.calculateOutput();

            telemetry.addData("Nectar Target Position", nectarTargetPositionTurret);
            telemetry.addData("Pollen Target Position", pollenTargetPositionTurret);
        }
    }

    public double getPollenTurretAngleDegrees(AnalogInput pollenTurretEncoder) {
        double voltage = pollenTurretEncoder.getVoltage();
        double rawDegrees = (voltage / 3.3) * 360.0;

        // Force the wrap-around to become negative degrees
        // Turning backwards past 0 (e.g., 350 degrees) becomes -10 degrees
        if (rawDegrees > 180.0) {
            rawDegrees -= 360.0;
        }

        return rawDegrees * ENCODER_TO_TURRET_RATIO;
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
