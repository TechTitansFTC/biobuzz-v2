package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.util.Constants.COLOR_SENSOR_PROXIMITY;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
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

    private final Telemetry telemetry;

    private final DcMotor transferMotor;

    public enum GamePieceColor {
        RED, BLUE, YELLOW, NONE, UNKNOWN
    }
    private final float[] hsvValues = new float[3];

    public StorageSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        sensor1 = hardwareMap.get(NormalizedColorSensor.class, "sensor1");

        sensor2 = hardwareMap.get(DigitalChannel.class, "sensor2");
        sensor2.setMode(DigitalChannel.Mode.INPUT);

        sensor3 = hardwareMap.get(NormalizedColorSensor.class, "sensor3");

        sensor4 = hardwareMap.get(DigitalChannel.class, "sensor4");
        sensor4.setMode(DigitalChannel.Mode.INPUT);

        sensor5 = hardwareMap.get(DigitalChannel.class, "sensor5");
        sensor5.setMode(DigitalChannel.Mode.INPUT);

        transferMotor = hardwareMap.get(DcMotor.class, "transferMotor");

        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        telemetry.addData("--- Distance/Color Sensors ---", "");

        telemetry.addData("Color Sensor 1", getSensor1Color());
        telemetry.addData("Distance Sensor 2", getSensor2Value());
        telemetry.addData("Color Sensor 3", getSensor3Color());
        telemetry.addData("Distance Sensor 4", getSensor4Value());
        telemetry.addData("Distance Sensor 5", getSensor5Value());

        telemetry.addData("", "");
        telemetry.addData("Transfer Motor Power", getTransferMotorPower());
    }

    public GamePieceColor getSensor1Color() {
        double distance = ((DistanceSensor) sensor1).getDistance(DistanceUnit.CM);;
        if (distance >= COLOR_SENSOR_PROXIMITY || Double.isNaN(distance)) {
            return GamePieceColor.NONE;
        }

        NormalizedRGBA colors = sensor1.getNormalizedColors();

        Color.colorToHSV(colors.toColor(), hsvValues);
        float hue = hsvValues[0];

        if (hue > 30 && hue < 90) {
            return GamePieceColor.YELLOW;
        } else if (hue > 200 && hue < 270) {
            return GamePieceColor.BLUE;
        } else if (hue < 20 || hue > 340) {
            return GamePieceColor.RED;
        }

        return GamePieceColor.UNKNOWN;
    }

    public boolean getSensor2Value() {
        return !sensor2.getState();
    }

    public GamePieceColor getSensor3Color() {
        double distance = ((DistanceSensor) sensor3).getDistance(DistanceUnit.CM);
        if (distance >= COLOR_SENSOR_PROXIMITY || Double.isNaN(distance)) {
            return GamePieceColor.NONE;
        }

        NormalizedRGBA colors = sensor3.getNormalizedColors();

        Color.colorToHSV(colors.toColor(), hsvValues);
        float hue = hsvValues[0];

        if (hue > 30 && hue < 90) {
            return GamePieceColor.YELLOW;
        } else if (hue > 200 && hue < 270) {
            return GamePieceColor.BLUE;
        } else if (hue < 20 || hue > 340) {
            return GamePieceColor.RED;
        }

        return GamePieceColor.UNKNOWN;
    }

    public boolean getSensor4Value() {
        return !sensor4.getState();
    }

    public boolean getSensor5Value() {
        return !sensor5.getState();
    }

    public void setTransferMotorPower(double power) {
        transferMotor.setPower(power);
    }

    public double getTransferMotorPower() {
        return transferMotor.getPower();
    }
}
