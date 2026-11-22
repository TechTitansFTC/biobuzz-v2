package org.firstinspires.ftc.teamcode.game;

import static org.firstinspires.ftc.teamcode.util.Constants.CONTROL_HUB_LOGO_FACING_DIRECTION;
import static org.firstinspires.ftc.teamcode.util.Constants.CONTROL_HUB_USB_FACING_DIRECTION;
import static org.firstinspires.ftc.teamcode.util.PoseStorage.autonomousEndPose;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.FlowerIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.ReverseCommand;
import org.firstinspires.ftc.teamcode.commands.PollenScoringCommand;
import org.firstinspires.ftc.teamcode.commands.StorageCommand;
import org.firstinspires.ftc.teamcode.subsystems.FlowerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ScoringSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.StorageSubsystem;
import org.firstinspires.ftc.teamcode.util.Constants;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp Main", group="Main")
public class TeleOp extends OpMode {
    MecanumDriveSubsystem mecanumDriveSubsystem;
    IntakeSubsystem intakeSubsystem;
    ScoringSubsystem scoringSubsystem;
    StorageSubsystem storageSubsystem;
    FlowerSubsystem flowerSubsystem;
    DriveCommand driveCommand;
    PollenScoringCommand pollenScoringCommand;
    StorageCommand storageCommand;
    FlowerIntakeCommand flowerIntakeCommand;
    ReverseCommand reverseCommand;
    IMU imu;
    GamepadEx gamepadEx;
    Follower follower;
    @Override
    public void init () {
        // Objects
        imu = hardwareMap.get(IMU.class,"imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(CONTROL_HUB_LOGO_FACING_DIRECTION, CONTROL_HUB_USB_FACING_DIRECTION)));

        gamepadEx = new GamepadEx(gamepad1);
        follower = org.firstinspires.ftc.teamcode.pedro.Constants.create(hardwareMap);
        follower.setPose(autonomousEndPose);

        // Subsystems
        mecanumDriveSubsystem = new MecanumDriveSubsystem(hardwareMap,imu,telemetry);
        intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry);
        scoringSubsystem = new ScoringSubsystem(hardwareMap, telemetry, follower);
        storageSubsystem = new StorageSubsystem(hardwareMap, telemetry);
        flowerSubsystem = new FlowerSubsystem(hardwareMap, telemetry);

        // Commands
        driveCommand = new DriveCommand(gamepadEx,mecanumDriveSubsystem);
        pollenScoringCommand = new PollenScoringCommand(scoringSubsystem, storageSubsystem, intakeSubsystem);
        storageCommand = new StorageCommand(storageSubsystem, intakeSubsystem);
        flowerIntakeCommand = new FlowerIntakeCommand(flowerSubsystem);
        reverseCommand = new ReverseCommand(storageSubsystem, intakeSubsystem);

        // Bindings
        mecanumDriveSubsystem.setDefaultCommand(driveCommand);
        storageSubsystem.setDefaultCommand(storageCommand);
        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(() -> mecanumDriveSubsystem.resetIMU());
        gamepadEx.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(pollenScoringCommand);
        gamepadEx.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(() -> scoringSubsystem.togglePollenShooter()); // flywheel on/off (on by default)
        gamepadEx.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenHeld(flowerIntakeCommand);
        gamepadEx.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenHeld(reverseCommand);
        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(() -> Constants.TEAM = "B");
        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(() -> Constants.TEAM = "R");
    }

    @Override
    public void loop () {
        CommandScheduler.getInstance().run();
        follower.update();
        telemetry.update();
        gamepadEx.readButtons();
    }

    @Override
    public void stop () {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().reset();
    }
}
