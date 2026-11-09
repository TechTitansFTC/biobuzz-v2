package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.util.Constants.BLUE_LOWER_BOUND;
import static org.firstinspires.ftc.teamcode.util.Constants.BLUE_UPPER_BOUND;
import static org.firstinspires.ftc.teamcode.util.Constants.COLOR_SENSOR_PROXIMITY;
import static org.firstinspires.ftc.teamcode.util.Constants.RED_LOWER_BOUND;
import static org.firstinspires.ftc.teamcode.util.Constants.RED_UPPER_BOUND;
import static org.firstinspires.ftc.teamcode.util.Constants.YELLOW_LOWER_BOUND;
import static org.firstinspires.ftc.teamcode.util.Constants.YELLOW_UPPER_BOUND;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class StorageSubsystem extends SubsystemBase {
    /*
    Sensor 1 - Closest to intake - Color
    Sensor 2 - Closest to Sensor 1 - Distance
    Sensor 3 - Inside Nectar Turret - Color
    Sensor 4 - Closest to Sensor 5 - Distance
    Sensor 5 - Inside Pollen Turret - Distance
     */

    private final NormalizedColorSensor sensor1;
    private final DigitalChannel sensor2;
    private final NormalizedColorSensor sensor3;
    private final DigitalChannel sensor4;
    private final DigitalChannel sensor5;
    private final Servo pollenGateServo;
    private final Servo nectarGateServo;
    private final Telemetry telemetry;

    private final DcMotor transferMotor;

    public enum GamePieceColor {
        RED, BLUE, YELLOW, NONE, UNKNOWN
    }

    public StorageSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        sensor1 = hardwareMap.get(NormalizedColorSensor.class, "sensor1"); // port 0 i2c expansion hub

        sensor2 = hardwareMap.get(DigitalChannel.class, "sensor2"); // port 0 digital expansion hub
        sensor2.setMode(DigitalChannel.Mode.INPUT);

        sensor3 = hardwareMap.get(NormalizedColorSensor.class, "sensor3"); // port 1 i2c expansion hub

        sensor4 = hardwareMap.get(DigitalChannel.class, "sensor4"); // port 2 digital expansion hub
        sensor4.setMode(DigitalChannel.Mode.INPUT);

        sensor5 = hardwareMap.get(DigitalChannel.class, "sensor5"); // port 4 digital expansion hub
        sensor5.setMode(DigitalChannel.Mode.INPUT);

        transferMotor = hardwareMap.get(DcMotor.class, "transfer"); // port 2 motor control hub
        transferMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        pollenGateServo = hardwareMap.get(Servo.class, "pollenGate"); // port 0 servo control hub
        nectarGateServo = hardwareMap.get(Servo.class, "nectarGate"); // port 0 servo expansion hub

        this.telemetry = telemetry;

        register();
    }

    @Override
    public void periodic() {
        telemetry.addData("--- Distance/Color Sensors ---", "");

        telemetry.addData("Color Sensor 1", getSensor1Color());
//        telemetry.addData("Color sensor 1 hue", getSensor1Hue());
        telemetry.addData("", "");
        telemetry.addData("Distance Sensor 2", getSensor2Value());
        telemetry.addData("Color Sensor 3", getSensor3Color());
        telemetry.addData("Distance Sensor 4", getSensor4Value());
        telemetry.addData("Distance Sensor 5", getSensor5Value());

        telemetry.addData("", "");
        telemetry.addData("Transfer Motor Power", getTransferMotorPower());
        telemetry.addData("Nectar Gate Position", getNectarGateServoPosition());
        telemetry.addData("Pollen Gate Position", getPollenGateServoPosition());
    }

    private GamePieceColor colorSensorToGamePiece(NormalizedColorSensor sensor) {
        double distance = ((DistanceSensor) sensor).getDistance(DistanceUnit.CM);
        if (distance >= COLOR_SENSOR_PROXIMITY || Double.isNaN(distance)) {
            return GamePieceColor.NONE;
        }

        NormalizedRGBA colors = sensor.getNormalizedColors();
        float[] hsvValues = new float[3];

        Color.colorToHSV(colors.toColor(), hsvValues);
        float hue = hsvValues[0];

        if (hue > YELLOW_LOWER_BOUND && hue < YELLOW_UPPER_BOUND) {
            return GamePieceColor.YELLOW;
        } else if (hue > BLUE_LOWER_BOUND && hue < BLUE_UPPER_BOUND) {
            return GamePieceColor.BLUE;
        } else if (hue < RED_UPPER_BOUND || hue > RED_LOWER_BOUND) {
            return GamePieceColor.RED;
        }

        return GamePieceColor.UNKNOWN;
    }

    public GamePieceColor getSensor1Color() {
        return colorSensorToGamePiece(sensor1);
    }

    public float getSensor1Hue() {
        double distance = ((DistanceSensor) sensor1).getDistance(DistanceUnit.CM);
        if (distance >= COLOR_SENSOR_PROXIMITY || Double.isNaN(distance)) {
            return -1;
        }

        float[] hsvValues = new float[3];
        NormalizedRGBA colors = sensor1.getNormalizedColors();

        Color.colorToHSV(colors.toColor(), hsvValues);
        float hue = hsvValues[0];

        return hue;

//        return GamePieceColor.UNKNOWN;
    }

    public double getSensor1Distance() {
        return ((DistanceSensor) sensor1).getDistance(DistanceUnit.CM);
    }

    public boolean getSensor2Value() {
        return sensor2.getState();
    }

    public GamePieceColor getSensor3Color() {
        return colorSensorToGamePiece(sensor3);
    }

    public boolean getSensor4Value() {
        return sensor4.getState();
    }

    public boolean getSensor5Value() {
        return sensor5.getState();
    }

    public void setTransferMotorPower(double power) {
        transferMotor.setPower(power);
    }

    public double getTransferMotorPower() {
        return transferMotor.getPower();
    }
    public void setPollenGateServoPosition(double position) {
        pollenGateServo.setPosition(position);
    }

    public void setNectarGateServoPosition(double position) {
        nectarGateServo.setPosition(position);
    }

    public double getPollenGateServoPosition() {
        return pollenGateServo.getPosition();
    }

    public double getNectarGateServoPosition() {
        return nectarGateServo.getPosition();
    }
}
