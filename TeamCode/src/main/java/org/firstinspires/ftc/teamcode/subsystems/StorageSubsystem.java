package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class StorageSubsystem extends SubsystemBase {
    /*
    Sensor 1 - Closest to intake - Color
    Sensor 2 - Closest to Sensor 1 - Distance
    Sensor 3 - Inside Nectar Turret - Color
    Sensor 4 - Closest to Sensor 5 - Distance
    Sensor 5 - Inside Pollen Turret - Distance
     */

    private final RevColorSensorV3 sensor1;
    private final DigitalChannel sensor2;
    private final RevColorSensorV3 sensor3;
    private final DigitalChannel sensor4;
    private final DigitalChannel sensor5;

    private final Telemetry telemetry;

    public StorageSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        sensor1 = hardwareMap.get(RevColorSensorV3.class, "sensor1");
        sensor2 = hardwareMap.get(DigitalChannel.class, "sensor2");
        sensor3 = hardwareMap.get(RevColorSensorV3.class, "sensor3");
        sensor4 = hardwareMap.get(DigitalChannel.class, "sensor4");
        sensor5 = hardwareMap.get(DigitalChannel.class, "sensor5");

        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {

    }

    public String getSensor1Color() {
//        if (sensor)
        return "";
    }
}
