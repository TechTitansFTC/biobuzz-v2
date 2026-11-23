package org.firstinspires.ftc.teamcode.game;


import static org.firstinspires.ftc.teamcode.util.Constants.APRIL_TAG_ALIGN_DETECTION_TIME;
import static org.firstinspires.ftc.teamcode.util.Constants.AUTO_INTAKE_TIME;
import static org.firstinspires.ftc.teamcode.util.Constants.FLOWER_LEFT_SERVO_DOWN;
import static org.firstinspires.ftc.teamcode.util.Constants.FLOWER_LEFT_SERVO_UP;
import static org.firstinspires.ftc.teamcode.util.Constants.FLOWER_RIGHT_SERVO_DOWN;
import static org.firstinspires.ftc.teamcode.util.Constants.FLOWER_RIGHT_SERVO_UP;
import static org.firstinspires.ftc.teamcode.util.Constants.INTAKE_MOTOR_POWER;
import static org.firstinspires.ftc.teamcode.util.Constants.PARK_TIME;
import static org.firstinspires.ftc.teamcode.util.Constants.SHOOTER_SPIN_UP_TIME;
import static org.firstinspires.ftc.teamcode.util.paths.PathChainBuilder.buildPathChain;

import java.util.Arrays;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.FlowerIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.PollenScoringCommand;
import org.firstinspires.ftc.teamcode.commands.ReverseCommand;
import org.firstinspires.ftc.teamcode.commands.ScoringCommand;
import org.firstinspires.ftc.teamcode.commands.StorageCommand;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.FlowerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ScoringSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.StorageSubsystem;
import org.firstinspires.ftc.teamcode.util.paths.AutonomousPaths;

@Autonomous(name = "Blue Auto", group="Main")
public class BlueAuto extends OpMode {
    IntakeSubsystem intakeSubsystem;
    ScoringSubsystem scoringSubsystem;
    StorageSubsystem storageSubsystem;
    PollenScoringCommand scoringCommand;
    FlowerIntakeCommand flowerIntakeCommand;
    FlowerSubsystem flowerSubsystem;
    StorageCommand storageCommand;
    private Follower follower;
    private SequentialCommandGroup autonomousRoutine;
    private AutonomousPaths autonomousPaths;
    private ElapsedTime autoTimer;
    boolean isShooting = false;
    private boolean tip = false; // down, true is up (down means down is correct scoring position, up means up is correct scoring position)
    private boolean knowTip = false;
    private boolean flowerUp = true; // true --> contains balls, false --> already intaked from
    private boolean flowerDown = true;
    private boolean garden = true;
    private boolean hasPollen = true;
    private ElapsedTime aprilTagTimer;
    private String middleDestination = ""; // flowerDown, garden
    private String centerDestination = ""; // up, down, flowerUp
    private double[] currPos;
    private enum AutoState {START, IDLE, SHOOT, INTAKE_1, INTAKE_2, MOVING, CHECK_TIP, PARK}
    private AutoState currentState;
    private boolean doneWithFirstMove = false;
    private boolean hasShot = false;
    int detectedAprilTag = 0;
    private boolean tipCheckedAtAll = false;
    private boolean visited = false;
    private boolean visted2 = false;
    double intakeTime = 0;

    @Override
    public void init() {
        autonomousPaths = new AutonomousPaths();
        org.firstinspires.ftc.teamcode.util.Constants.TEAM = "B";


        follower = Constants.create(hardwareMap);
        follower.setX(autonomousPaths.getInitial()[0]);
        follower.setY(autonomousPaths.getInitial()[1]);
        follower.setHeading(Math.toRadians(autonomousPaths.getInitial()[2]));
        currPos = autonomousPaths.getInitial();

        intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry);
        scoringSubsystem = new ScoringSubsystem(hardwareMap, telemetry, follower);
        storageSubsystem = new StorageSubsystem(hardwareMap, telemetry);
        flowerSubsystem = new FlowerSubsystem(hardwareMap, telemetry);
        scoringCommand = new PollenScoringCommand(scoringSubsystem, storageSubsystem, intakeSubsystem);
        flowerIntakeCommand = new FlowerIntakeCommand(flowerSubsystem);
        storageCommand = new StorageCommand(storageSubsystem, intakeSubsystem);

        aprilTagTimer = new ElapsedTime();
        aprilTagTimer.reset();
        autoTimer = new ElapsedTime();
        autoTimer.reset();

        storageCommand.schedule();



    }

    @Override
    public void start() {
        currentState = AutoState.START;
//        flowerSubsystem.setLeftServoPosition(FLOWER_LEFT_SERVO_UP);
        flowerSubsystem.setRightServoPosition(FLOWER_RIGHT_SERVO_UP);
        autoTimer.reset();
    }

    @Override
    public void loop() {
        follower.update();
        CommandScheduler.getInstance().run();
        switch (currentState) {
            case START:
                follower.follow(buildPathChain(autonomousPaths.getInitial(), autonomousPaths.getDown()));
                currentState = AutoState.MOVING;
                currPos = autonomousPaths.getDown();
                break;
            case IDLE:
                if (30 - autoTimer.seconds() <= PARK_TIME) {
                    if (Arrays.equals(currPos, autonomousPaths.getDown())) {
                        follower.follow(buildPathChain(currPos, autonomousPaths.getPark(), autonomousPaths.getDownParkControl()));
                    } else {
                        follower.follow(buildPathChain(currPos, autonomousPaths.getPark()));
                    }
                    currPos = autonomousPaths.getPark();
                    currentState = AutoState.PARK;
                    break;
                }
                if (Arrays.equals(currPos, autonomousPaths.getFlowerUp1()) || Arrays.equals(currPos, autonomousPaths.getFlowerDown1()) || Arrays.equals(currPos, autonomousPaths.getGarden1())) {
                    currentState = AutoState.INTAKE_1;
                    break;
                }
                if (hasPollen) {
                    visted2 = true;
                    if (knowTip) {
                        hasPollen = false;
                        currentState = AutoState.SHOOT;
                        break;
                    } else {
                        if (Arrays.equals(currPos, autonomousPaths.getDown()) || Arrays.equals(currPos, autonomousPaths.getUp())) {
                            visited = true;
                            aprilTagTimer.reset();
                            currentState = AutoState.CHECK_TIP;
                            break;
                        } else {
                            if (Arrays.equals(currPos, autonomousPaths.getFlowerUp1()) || Arrays.equals(currPos, autonomousPaths.getFlowerUp2())) {
                                follower.follow(buildPathChain(currPos, autonomousPaths.getUp()));
                                currPos = autonomousPaths.getUp();
                            } else {
                                follower.follow(buildPathChain(currPos, autonomousPaths.getDown()));
                                currPos = autonomousPaths.getDown();
                            }
                            currentState = AutoState.MOVING;
                        }
                        break;
                    }
                }
                if (garden) {
                    garden = false;
                    if (Arrays.equals(currPos, autonomousPaths.getUp())) {
                        middleDestination = "garden";
                        follower.follow(buildPathChain(currPos, autonomousPaths.getGarden1(), autonomousPaths.getUpGardenControl()));
                    } else {
                        follower.follow(buildPathChain(currPos, autonomousPaths.getGarden1()));
                    }
                    currPos = autonomousPaths.getGarden1();
                    currentState = AutoState.MOVING;
                    break;
                }
                if (flowerUp) {
                    flowerUp = false;
                    if (Arrays.equals(currPos, autonomousPaths.getDown())) {
                        follower.follow(buildPathChain(currPos, autonomousPaths.getFlowerUp1(), autonomousPaths.getDownFlowerUpControl()));
                    } else {
                        follower.follow(buildPathChain(currPos, autonomousPaths.getFlowerUp1()));
                    }
                    currPos = autonomousPaths.getFlowerUp1();
                    currentState = AutoState.MOVING;
                    break;
                }
                if (flowerDown) {
                    flowerDown = false;
                    if (Arrays.equals(currPos, autonomousPaths.getUp())) {
                        follower.follow(buildPathChain(currPos, autonomousPaths.getFlowerDown1(), autonomousPaths.getUpFlowerDownControl()));
                    } else {
                        follower.follow(buildPathChain(currPos, autonomousPaths.getFlowerDown1()));
                    }
                    currPos = autonomousPaths.getFlowerDown1();
                    currentState = AutoState.MOVING;
                    break;
                }
                break;
            case SHOOT:
                if (autoTimer.seconds() <= SHOOTER_SPIN_UP_TIME) break;
                if (!isShooting) {
                    hasPollen = false;
                    knowTip = false;
                    hasShot = true;
                    CommandScheduler.getInstance().schedule(scoringCommand);
                    isShooting = true;
                }
                if (!CommandScheduler.getInstance().isScheduled(scoringCommand)) {
                    isShooting = false;
                    currentState = AutoState.IDLE;
                }
                break;
            case INTAKE_1: // Assumes that intake transition time < time to go from intake pos 1 to 2
//                intakeSubsystem.setWheelPivotPosition(intakeWheelPivotPositionUp);
//                intakeSubsystem.setRampPivotPosition(intakeRampPivotPositionUp);
                intakeSubsystem.setMotorPower(INTAKE_MOTOR_POWER);
                if (Arrays.equals(currPos, autonomousPaths.getFlowerDown1()) || Arrays.equals(currPos, autonomousPaths.getFlowerUp1())) {
                    flowerSubsystem.setRightServoPosition(FLOWER_RIGHT_SERVO_DOWN);
//                    flowerSubsystem.setLeftServoPosition(FLOWER_LEFT_SERVO_DOWN);
                } else {
                    flowerSubsystem.setRightServoPosition(FLOWER_RIGHT_SERVO_UP);
//                    flowerSubsystem.setLeftServoPosition(FLOWER_LEFT_SERVO_UP);
                }
                if (Arrays.equals(currPos, autonomousPaths.getFlowerDown1())) {
                    follower.follow(buildPathChain(currPos, autonomousPaths.getFlowerDown2(),   10));
                    currPos = autonomousPaths.getFlowerDown2();
                } else if (Arrays.equals(currPos, autonomousPaths.getFlowerUp1())) {
                    follower.follow(buildPathChain(currPos, autonomousPaths.getFlowerUp2()));
                    currPos = autonomousPaths.getFlowerUp2();
                } else if (Arrays.equals(currPos, autonomousPaths.getGarden1())) {
                    follower.follow(buildPathChain(currPos, autonomousPaths.getGarden2()));
                    currPos = autonomousPaths.getGarden2();
                }
                currentState = AutoState.INTAKE_2;
                intakeTime = autoTimer.seconds();
                break;
            case INTAKE_2:
                if (autoTimer.seconds() - intakeTime >= AUTO_INTAKE_TIME) {
                    hasPollen = true;
                    flowerSubsystem.setRightServoPosition(FLOWER_RIGHT_SERVO_UP);
                    currentState = AutoState.IDLE;
                }
                break;
            case MOVING:
                if (!follower.isBusy()) {
                    currentState = AutoState.IDLE;
                    doneWithFirstMove = true;
                }
                break;
            case CHECK_TIP:
                tipCheckedAtAll = true;
                boolean tagVisible = scoringSubsystem.getBearing() != 361.0;

                if (Arrays.equals(currPos, autonomousPaths.getDown())) {
                    if (tagVisible) {
                        tip = false;
                        knowTip = true;
                        currentState = AutoState.IDLE;
                    } else if (aprilTagTimer.seconds() >= APRIL_TAG_ALIGN_DETECTION_TIME) {
                        follower.follow(buildPathChain(currPos, autonomousPaths.getUp(), autonomousPaths.getUpDownControl()));
                        currPos = autonomousPaths.getUp();
                        currentState = AutoState.MOVING;
                    }
                } else if (Arrays.equals(currPos, autonomousPaths.getUp())) {
                    if (tagVisible) {
                        tip = true;
                        knowTip = true;
                        currentState = AutoState.IDLE;
                    } else if (aprilTagTimer.seconds() >= APRIL_TAG_ALIGN_DETECTION_TIME) {
                        follower.follow(buildPathChain(currPos, autonomousPaths.getDown(), autonomousPaths.getUpDownControl()));
                        currPos = autonomousPaths.getDown();
                        currentState = AutoState.MOVING;
                    }
                } else {
                    currentState = AutoState.IDLE;
                }
                break;
            case PARK:
                if (!follower.isBusy()) {
                    requestOpModeStop();
                    return;
                }
                break;
        }

        telemetry.addData("", "");
        telemetry.addData("State", currentState);
        telemetry.addData("tip", tip);
        telemetry.addData("knowTip", knowTip);
        telemetry.addData("flowerUp", flowerUp);
        telemetry.addData("flowerDown", flowerDown);
        telemetry.addData("garden", garden);
        telemetry.addData("hasPollen", hasPollen);
        telemetry.addData("doneWithFirstMove", doneWithFirstMove);
        telemetry.addData("currPos", autonomousPaths.getPose(currPos));
        telemetry.addData("lastDetectedapriltag", detectedAprilTag);
        telemetry.addData("hasShot", hasShot);
        telemetry.addData("tipChecked", tipCheckedAtAll);
        telemetry.addData("tipchecked0", visited);
        telemetry.addData("haspollen1", visted2);
        telemetry.update();
    }

    @Override
    public void stop() {
        CommandScheduler.getInstance().reset();
    }
}