package org.firstinspires.ftc.teamcode.game;

import static org.firstinspires.ftc.teamcode.util.Constants.CONTROL_HUB_LOGO_FACING_DIRECTION;
import static org.firstinspires.ftc.teamcode.util.Constants.CONTROL_HUB_USB_FACING_DIRECTION;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp Main")
public class TeleOp extends OpMode {
    MecanumDriveSubsystem mecanumDriveSubsystem;
    DriveCommand driveCommand;
    IMU imu;
    GamepadEx gamepadEx;
    @Override
    public void init () {
        // Objects
        imu = hardwareMap.get(IMU.class,"imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(CONTROL_HUB_LOGO_FACING_DIRECTION, CONTROL_HUB_USB_FACING_DIRECTION)));

        gamepadEx = new GamepadEx(gamepad1);

        // Subsystems
        mecanumDriveSubsystem = new MecanumDriveSubsystem(hardwareMap,imu,telemetry);

        // Commands
        driveCommand = new DriveCommand(gamepadEx,mecanumDriveSubsystem);

        // Bindings
        mecanumDriveSubsystem.setDefaultCommand(driveCommand);
        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(() -> mecanumDriveSubsystem.resetIMU());
    }

    @Override
    public void loop () {
        CommandScheduler.getInstance().run();
        gamepadEx.readButtons();
    }

    @Override
    public void stop () {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().reset();
    }
}
