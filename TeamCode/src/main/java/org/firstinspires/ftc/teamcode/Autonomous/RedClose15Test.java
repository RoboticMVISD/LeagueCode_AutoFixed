package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Teleop.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.Teleop.SubSystems.MovementSystems.Movement;
import org.firstinspires.ftc.teamcode.Teleop.SubSystems.Shooter;
import org.firstinspires.ftc.teamcode.Autonomous.pedroPathing.Constants;

@Autonomous(name = "RedClose15Test")
public class RedClose15Test extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;
    private Servo turretRotator;

    private int timesRowIntaken;

    private enum StageOne {
        DRIVE_SET_UP_PRELOAD_SHOOT, SHOOT_PRELOAD, DRIVE_ROW_ONE, INTAKE_ROW_ONE, SHOOT_ROW_ONE,
        DRIVE_ROW_TWO, INTAKE_ROW_TWO, DRIVE_BACKUP_FROM_TWO, DRIVE_SHOOT_ROW_TWO, SHOOT_ROW_TWO,
        DRIVE_PREINTAKE_RAMP, DRIVE_SET_UP_RAMP, INTAKE_RAMP, DRIVE_RESET, DRIVE_SHOOT_RAMP_POSITION, SHOOT_RAMP,
        DRIVE_PARK, DRIVE_RESET_BACK,DONE
    }
    private StageOne currentStage;

    // --- Poses ---
    private final Pose startPose = new Pose(127.07, 109.93, Math.toRadians(0));
    private final Pose shootPose = new Pose(102.23, 98.03, Math.toRadians(44));
    private final Pose rowOneStart = new Pose(99.82, 80.58, Math.toRadians(0));
    private final Pose rowOneEnd = new Pose(124.50, 80.58, Math.toRadians(0));
    private final Pose rowTwoStart = new Pose(99.76, 57.16, Math.toRadians(0));
    private final Pose rowTwoEnd = new Pose(132.05, 57.19, Math.toRadians(0));
    private final Pose backUpFromTwo = new Pose(123, 56, Math.toRadians(0));
    private final Pose parkPose = new Pose(94.72, 63.82, Math.toRadians(270));
    private final Pose preIntakeRampPose = new Pose(110,60, Math.toRadians(0));
    private final Pose setUpIntakeRamp = new Pose(128,56, Math.toRadians(20));
    private final Pose intakeRampPose = new Pose(132,52, Math.toRadians(60));

    // --- Paths ---
    private PathChain shootPreloadPath, driveFirstRowPath, intakeFirstRowPath, driveResetBackRowOnePath;
    private PathChain driveRowTwoPath, intakeRowTwoPath, backUpFromRowTwoPath, goToShootRowTwo;
    private PathChain drivePreIntakeRampPath, setUpRampPath, intakeRampPath, driveReset, shootRampPath, driveParkPath;

    // Shooter constants
    private static final double MEDIUM_SHOOT_SPEED = Shooter.SPIN_UP_VELOCITY_MEDIUMRANGE - 50;
    private static final double VELOCITY_TOLERANCE = 20;
    private static final double INTAKE_SPEED = .8;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);

        Intake.init(this);
        Shooter.init(this, false);
        Movement.init(hardwareMap);

        turretRotator = hardwareMap.servo.get("turretRotator");
        turretRotator.setDirection(Servo.Direction.REVERSE);

        pathTimer = new Timer();
        opModeTimer = new Timer();
        timesRowIntaken = 0;

        buildPaths();
        currentStage = StageOne.SHOOT_PRELOAD;
        pathTimer.resetTimer();
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        updateAutoStage();
        updateTelemetry();
    }

    private PathChain buildPath(Pose start, Pose end) {
        return follower.pathBuilder()
                .addPath(new BezierLine(start, end))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .build();
    }

    private void buildPaths() {
        shootPreloadPath = buildPath(startPose, shootPose);
        driveFirstRowPath = buildPath(shootPose, rowOneStart);
        intakeFirstRowPath = buildPath(rowOneStart, rowOneEnd);
        driveResetBackRowOnePath = buildPath(rowOneEnd, shootPose);

        driveRowTwoPath = buildPath(shootPose, rowTwoStart);
        intakeRowTwoPath = buildPath(rowTwoStart, rowTwoEnd);
        backUpFromRowTwoPath = buildPath(rowTwoEnd, backUpFromTwo);
        goToShootRowTwo = buildPath(backUpFromTwo, shootPose);

        drivePreIntakeRampPath = buildPath(shootPose, preIntakeRampPose);
        setUpRampPath = buildPath(preIntakeRampPose, setUpIntakeRamp);
        intakeRampPath = buildPath(setUpIntakeRamp, intakeRampPose);
        driveReset = buildPath(intakeRampPose, preIntakeRampPose);
        shootRampPath = buildPath(preIntakeRampPose, shootPose);

        driveParkPath = buildPath(shootPose, parkPose);
    }

    private void updateAutoStage() {
        switch(currentStage) {
            case DRIVE_SET_UP_PRELOAD_SHOOT:
                Shooter.setShooterPower(MEDIUM_SHOOT_SPEED);
                follower.followPath(shootPreloadPath);
            case SHOOT_PRELOAD:
                if (!follower.isBusy())
                    runShooter(pathTimer, 3);
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3)
                    advanceStage(StageOne.DRIVE_ROW_ONE);
                break;

            case DRIVE_ROW_ONE:
                if (!follower.isBusy())
                    follower.followPath(driveFirstRowPath, true);
                    advanceStage(StageOne.INTAKE_ROW_ONE);
                break;

            case INTAKE_ROW_ONE:
                if (!follower.isBusy()) {
                    Intake.intake.setPower(INTAKE_SPEED);
                    follower.followPath(intakeFirstRowPath, true);
                    advanceStage(StageOne.DRIVE_RESET_BACK);
                }
                break;

            case DRIVE_RESET_BACK:
                if (!follower.isBusy()) {
                    Intake.intake.setPower(INTAKE_SPEED);
                    Shooter.setShooterPower(MEDIUM_SHOOT_SPEED);
                    follower.followPath(driveResetBackRowOnePath, true);
                    advanceStage(StageOne.SHOOT_ROW_ONE);
                }
                break;

            case SHOOT_ROW_ONE:
                if (!follower.isBusy())
                    runShooter(pathTimer, 4);
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 4)
                    turnOffSystems();
                    advanceStage(StageOne.DRIVE_ROW_TWO);
                break;

            case DRIVE_ROW_TWO:
                if (!follower.isBusy()) {
                    follower.followPath(driveRowTwoPath, true);
                    advanceStage(StageOne.INTAKE_ROW_TWO);
                }
                break;

            case INTAKE_ROW_TWO:
                if (!follower.isBusy()) {
                    Intake.intake.setPower(INTAKE_SPEED);
                    follower.followPath(intakeRowTwoPath, true);
                    advanceStage(StageOne.DRIVE_BACKUP_FROM_TWO);
                }
                break;

            case DRIVE_BACKUP_FROM_TWO:
                if (!follower.isBusy()) {
                    Intake.intake.setPower(INTAKE_SPEED);
                    follower.followPath(backUpFromRowTwoPath, true);
                    advanceStage(StageOne.DRIVE_SHOOT_ROW_TWO);
                }
                break;
            case DRIVE_SHOOT_ROW_TWO:
                if (!follower.isBusy()){
                    Shooter.setShooterPower(MEDIUM_SHOOT_SPEED);
                     Intake.intake.setPower(INTAKE_SPEED);
                     follower.followPath(goToShootRowTwo);
                     advanceStage(StageOne.SHOOT_ROW_TWO);
                } break;
            case SHOOT_ROW_TWO:
                if (!follower.isBusy()) {
                    runShooter(pathTimer, 4);
                }
                if (pathTimer.getElapsedTimeSeconds() > 4) {
                    turnOffSystems();
                    advanceStage(StageOne.DRIVE_PREINTAKE_RAMP);
                }
                break;

            case DRIVE_PREINTAKE_RAMP:
                if (!follower.isBusy()) {
                    follower.followPath(drivePreIntakeRampPath, true);
                    advanceStage(StageOne.INTAKE_RAMP);
                }
                break;
            case DRIVE_SET_UP_RAMP:
                if (!!follower.isBusy()){
                    follower.followPath(setUpRampPath);
                    advanceStage(StageOne.INTAKE_RAMP);
                }
            case INTAKE_RAMP:
                if (!follower.isBusy()) {
                    Intake.intake.setPower(INTAKE_SPEED);
                    follower.followPath(intakeRampPath, true);
                }
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 4){
                    advanceStage(StageOne.DRIVE_RESET);
                }
                break;

            case DRIVE_RESET:
                if (!follower.isBusy()) {
                    follower.followPath(driveReset, true);
                    advanceStage(StageOne.SHOOT_RAMP);
                }
                break;
//-----//
            case DRIVE_SHOOT_RAMP_POSITION:
                if (!follower.isBusy()){
                    Shooter.setShooterPower(MEDIUM_SHOOT_SPEED);
                    follower.followPath(shootRampPath);
                    advanceStage(StageOne.SHOOT_RAMP);
                }
            case SHOOT_RAMP:
                if (!follower.isBusy()){
                    runShooter(pathTimer, 4);
                }
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 4) {
                    timesRowIntaken++;
                    if (timesRowIntaken < 3){
                        advanceStage(StageOne.DRIVE_PREINTAKE_RAMP);
                    } else if (timesRowIntaken == 3){
                        advanceStage(StageOne.DRIVE_PARK);
                    }
                }
                break;

            case DRIVE_PARK:
                if (!follower.isBusy()) follower.followPath(driveParkPath, true);
                advanceStage(StageOne.DONE);
                break;

            case DONE:
                turnOffSystems();
                break;
        }
    }

    private void advanceStage(StageOne next) {
        currentStage = next;
        pathTimer.resetTimer();
    }

    // --- Shooter Helper ---
    private void runShooter(Timer timer, double durationSeconds) {
        Shooter.setShooterPower(MEDIUM_SHOOT_SPEED, 0.48, turretRotator);
        if (Shooter.rightShooter.getVelocity() > MEDIUM_SHOOT_SPEED - VELOCITY_TOLERANCE &&
                Shooter.rightShooter.getVelocity() < MEDIUM_SHOOT_SPEED + VELOCITY_TOLERANCE) {
            Intake.setBothIntakePower(1);
        }
        if (timer.getElapsedTimeSeconds() > durationSeconds) turnOffSystems();
    }

    private void turnOffSystems() {
        Shooter.setShooterPower(0);
        Intake.setBothIntakePower(0);
    }

    private void updateTelemetry() {
        telemetry.addData("Auto Stage", currentStage);
        telemetry.addData("Shooter Velocity", Shooter.rightShooter.getVelocity());
        telemetry.addData("Path Timer", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("OpMode Timer", opModeTimer.getElapsedTimeSeconds());
        telemetry.update();
    }
}
