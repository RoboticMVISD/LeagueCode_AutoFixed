package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Teleop.SubSystems.CameraSystems.AutoAim_Distance;
import org.firstinspires.ftc.teamcode.Teleop.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.testAndOldClasses.Movement;
import org.firstinspires.ftc.teamcode.Teleop.SubSystems.Shooter;
import org.firstinspires.ftc.teamcode.Autonomous.pedroPathing.Constants;


//TIMES:
@Autonomous (name = "RedAuto15")
public class Red15Ball extends OpMode{

    /*
    How this auto will work code wise is that it will have three separate methods which hold case statements.
    This is to symbolize the stages of the auto and what each stage completes/does.
     */
    private Follower follower;
    public static Servo turretRotator;
    private static AutoAim_Distance autoAimDistance = new AutoAim_Distance();



    //Variables & Enum For Shooting/Getting Preload and First Row of Balls
    private enum PathStateOne {
        DRIVE_GETTING_INTO_SHOOT_POS, SHOOT_PRELOAD,
        DRIVE_1ST_ROW_POS, HIT_LEVER, SET_UP_LEVER_HIT, DRIVE_RESET_MID_ONE, SHOOT_FIRST_ROW,
        DRIVE_2ND_ROW_POS, INTAKE_2ND_ROW, DRIVE_BACK_ROW_TWO, DRIVE_RESET_MID_TWO, SHOOT_SECOND_ROW,
        DRIVE_3RD_ROW_POS, DRIVE_RESET_MID_THREE, INTAKE_THIRD_ROW, SHOOT_THIRD_ROW,
        DRIVE_PREINTAKE_RAMP, DRIVE_SET_UP_RAMP, INTAKE_RAMP, DRIVE_RESET, DRIVE_SHOOT_RAMP_POSITION, SHOOT_RAMP,
        DRIVE_PARK;

    }
    private PathStateOne pathStateOne;
    private final Pose startPose = new Pose(127.073732718894, 109.92626728110596, Math.toRadians(0));
    private final Pose shootPose = new Pose(94.83926031294453, 91.76671408250354, Math.toRadians(37));
    private final Pose shootPoseEX = new Pose(94.839, 91.767, Math.toRadians(39));
    private final Pose preHitLever = new Pose(120, 70.580, Math.toRadians(0));
    private final Pose hitLeverPose = new Pose(127, 71.380, Math.toRadians(0));
    private final Pose rowOneStart = new Pose(99.820, 80.57972350230413, Math.toRadians(0));
    private final Pose rowOneEnd = new Pose(126.50241820768138, 80.57972350230413  , Math.toRadians(0));
    private PathChain shootFirstThree, getIntoRowOnePos, getFirstRow, resetBackOne, hitLever, readyToHitLever;



    //Variables To Shoot Second Row
    private final Pose rowTwoStart = new Pose(99.76036866359446, 56.866359447004605, Math.toRadians(0));
    private final Pose rowTwoEnd = new Pose(134.54608294930875, 56.39631336405527, Math.toRadians(0));
    private  PathChain getIntoRowTwo, getRowTwo, resetBackTwo,backUpRowTwo;




    //Variables to shoot 3rd Row and Park

    private final Pose rowThreeStart = new Pose(99.76036866359446, 33.5529953917, Math.toRadians(0));
    private final Pose rowThreeEnd = new Pose(134.04608294930875, 33.5529953917, Math.toRadians(0));
    private final Pose backUpSpot = new Pose(126.04878048780488, 33.5529953917, Math.toRadians(0));
    private final Pose parkPose = new Pose(120, 70.57972350230413, Math.toRadians(0));
    private PathChain getIntoRowThree, getRowThree,resetBackThree, park;

    //Ramp Variables
    private final Pose preIntakeRampPose = new Pose(110,60, Math.toRadians(0));
    private final Pose setUpIntakeRamp = new Pose(128,57, Math.toRadians(20));
    private final Pose intakeRampPose = new Pose(133,53, Math.toRadians(57));
    private PathChain drivePreIntakeRampPath, setUpRampPath, intakeRampPath, driveReset, shootRampPath, driveParkPath;


    private Timer pathTimer, opModeTimer;
    private static final double SHOOT_SPEED = Shooter.SPIN_UP_VELOCITY_MEDIUMRANGE;
    private static final double VELOCITY_TOLERANCE = 20;
    private int timesRowIntaken;



    private PathChain buildPath(Pose start, Pose end){
        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(start, end))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .build();
        return path;
    }

    private PathChain buildPath(Pose start, Pose end, Boolean isTangential){
        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(start, end))
                .setTangentHeadingInterpolation()
                .build();
        return path;
    }
    private void buildPaths(){
        //For Preload and Row One
        shootFirstThree = buildPath(startPose, shootPose);
        getIntoRowOnePos = buildPath(shootPose, rowOneStart);
        getFirstRow = buildPath(rowOneStart, rowOneEnd);
        readyToHitLever = buildPath(rowOneEnd, preHitLever);
        hitLever = buildPath(preHitLever, hitLeverPose);
        resetBackOne = buildPath(hitLeverPose, shootPoseEX);

        //For Row Two
        getIntoRowTwo = buildPath(shootPose, rowTwoStart);
        getRowTwo = buildPath(rowTwoStart, rowTwoEnd);
        backUpRowTwo = buildPath(rowTwoEnd, rowTwoStart);
        resetBackTwo = buildPath(rowTwoStart, shootPoseEX);

        //For Row Three and Park
        getIntoRowThree = buildPath(shootPose,rowThreeStart);
        getRowThree = buildPath(rowThreeStart, rowThreeEnd);
        resetBackThree = buildPath(rowThreeEnd, shootPoseEX);
        park = buildPath(shootPose, parkPose);

        //For Ramp Intaking
        drivePreIntakeRampPath = buildPath(shootPose, preIntakeRampPose);
        setUpRampPath = buildPath(preIntakeRampPose, setUpIntakeRamp);
        intakeRampPath = buildPath(setUpIntakeRamp, intakeRampPose);
        driveReset = buildPath(intakeRampPose, shootPose);
        shootRampPath = buildPath(preIntakeRampPose, shootPose);
    }

    //Method/Switch statement for shooting preload and getting row one & shooting. WORKS
    private void autoCases(){
        switch (pathStateOne) {
            case DRIVE_GETTING_INTO_SHOOT_POS:
                //Works
                follower.followPath(shootFirstThree, true);
                AutoAim_Distance.setLauncherPower();
                break;
            case SHOOT_PRELOAD: //Works
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() < 3.3){
                    shoot();
                } else if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3.3){
                    turnOffSystems();
                    setPathStateOne(PathStateOne.DRIVE_2ND_ROW_POS);
                } break;
            case DRIVE_2ND_ROW_POS:  //Works
                if (!follower.isBusy()){
                    turnOffSystems();
                    follower.followPath(getIntoRowTwo, true);
                    setPathStateOne(PathStateOne.INTAKE_2ND_ROW);
                } break;
            case INTAKE_2ND_ROW:
                if (!follower.isBusy()){
                    intakeBalls(pathTimer);
                    follower.followPath(getRowTwo, true);
                    setPathStateOne(PathStateOne.DRIVE_BACK_ROW_TWO);
                } break;
            case DRIVE_BACK_ROW_TWO:
                if (!follower.isBusy()){
                    follower.followPath(backUpRowTwo, true);
                    setPathStateOne(PathStateOne.DRIVE_RESET_MID_TWO);
                }break;
            case DRIVE_RESET_MID_TWO:
                if (!follower.isBusy()) {
                    Shooter.setShooterPower(Shooter.SPIN_UP_VELOCITY_MEDIUMRANGE - 50);
                    follower.followPath(resetBackTwo, true);
                    Shooter.setShooterPower(Shooter.SPIN_UP_VELOCITY_MEDIUMRANGE - 50);
                    setPathStateOne(PathStateOne.SHOOT_SECOND_ROW);
                } break;
            case SHOOT_SECOND_ROW:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() < 3.3){
                    shoot();
                } else if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3.3){
                    turnOffSystems();
                    setPathStateOne(PathStateOne.DRIVE_PREINTAKE_RAMP);
                } break;
            case DRIVE_PREINTAKE_RAMP:
                if (!follower.isBusy()) {
                    follower.followPath(drivePreIntakeRampPath, true);
                    setPathStateOne(PathStateOne.DRIVE_SET_UP_RAMP);
                }
                break;
            case DRIVE_SET_UP_RAMP:
                if (!follower.isBusy()){
                    follower.followPath(setUpRampPath, true);
                    setPathStateOne(PathStateOne.INTAKE_RAMP);
                }
            case INTAKE_RAMP:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() < 2.7) {
                    Intake.intake.setPower(1);
                    follower.followPath(intakeRampPath, true);
                } else if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.7){
                    setPathStateOne(PathStateOne.DRIVE_RESET);
                }
                break;
            case DRIVE_RESET:
                if (!follower.isBusy()) {
                    Shooter.setShooterPower(SHOOT_SPEED);
                    follower.followPath(driveReset, true);
                    setPathStateOne(PathStateOne.DRIVE_SHOOT_RAMP_POSITION);
                }
                break;
            case DRIVE_SHOOT_RAMP_POSITION:
                if (!follower.isBusy()){
                    turnOffSystems();
                    setPathStateOne(PathStateOne.SHOOT_RAMP);
                }
            case SHOOT_RAMP:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() < 2.5){
                    shootEX();
                } else if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.5) {
                timesRowIntaken++;
                    if (timesRowIntaken < 2){
                        turnOffSystems();
                        setPathStateOne(PathStateOne.DRIVE_PREINTAKE_RAMP);
                    } else if (timesRowIntaken == 2){
                        turnOffSystems();
                        setPathStateOne(PathStateOne.DRIVE_1ST_ROW_POS);
                    }
                } break;
            case DRIVE_1ST_ROW_POS: //Works
                if (!follower.isBusy()){
                    intakeBalls(pathTimer);
                    follower.followPath(getFirstRow, true);
                    setPathStateOne(PathStateOne.DRIVE_RESET_MID_ONE);
                } break;
            case DRIVE_RESET_MID_ONE: // Works
                if (!follower.isBusy()) {
                    Shooter.setShooterPower(Shooter.SPIN_UP_VELOCITY_MEDIUMRANGE - 50);//Works
                    follower.followPath(resetBackOne, true);
                    Shooter.setShooterPower(Shooter.SPIN_UP_VELOCITY_MEDIUMRANGE - 50);//Works
                    setPathStateOne(PathStateOne.SHOOT_FIRST_ROW);
                } break;
            case SHOOT_FIRST_ROW: //Works
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() < 3){
                    shootEX();
                } else if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3){
                    turnOffSystems();
                    setPathStateOne(PathStateOne.DRIVE_PARK);
                }
                break;
            case DRIVE_PARK:
                telemetry.addLine("All Paths Done");
                break;
            default:
                turnOffSystems();
                break;
        }
    }

    private void setPathStateOne(PathStateOne newState) {
        pathStateOne = newState;
        pathTimer.resetTimer();
    }
    private void shoot() {
        Shooter.setShooterPower(SHOOT_SPEED, .33, turretRotator);

        if (Shooter.rightShooter.getVelocity() > SHOOT_SPEED - 40 && Shooter.rightShooter.getVelocity() < SHOOT_SPEED + 40){
            Intake.setBothIntakePower(1);
        }

    }

    private void shootEX() {
       Shooter.setShooterPower(SHOOT_SPEED, .2, turretRotator);

        if (Shooter.rightShooter.getVelocity() > SHOOT_SPEED - 40 && Shooter.rightShooter.getVelocity() < SHOOT_SPEED + 40){
            Intake.setBothIntakePower(1);
        }

    }

    public void disableAutoShoot(){
        AutoAim_Distance.aimEnabled = false;
        AutoAim_Distance.launcherRequested = false;
        turnOffSystems();
    }

    public void enableAutoShoot(){
        AutoAim_Distance.aimEnabled = true;
        AutoAim_Distance.launcherRequested = true;
        double targetSpeed = AutoAim_Distance.targetLaunchVelocity;

        if (Shooter.rightShooter.getVelocity() < targetSpeed + VELOCITY_TOLERANCE
            && Shooter.rightShooter.getVelocity() > targetSpeed - VELOCITY_TOLERANCE){
            Intake.setBothIntakePower(1);
        }
    }
    private void turnOffSystems() {
        Shooter.setShooterPower(0);
        Intake.setBothIntakePower(0);
    }
    public void intakeBalls(Timer time){
        Intake.intake.setPower(.8);
    }





    public void init() {
        pathStateOne = PathStateOne.DRIVE_GETTING_INTO_SHOOT_POS;

        pathTimer = new Timer();
        opModeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);

        Intake.init(this);
        Shooter.init(this, false);
        Movement.init(this.hardwareMap);
        autoAimDistance.init(this);
        AutoAim_Distance.aimEnabled = false;
        AutoAim_Distance.launcherRequested = false;

        turretRotator = this.hardwareMap.servo.get("turretRotator");
        turretRotator.setDirection(Servo.Direction.REVERSE);

        buildPaths();
        follower.setPose(startPose);

        timesRowIntaken = 0;

    }

    public void start(){
        opModeTimer.resetTimer();
    }

    public void loop(){
        //autoAimDistance.loop(true);

        follower.update();
        autoCases();
        updateTelemetry();
    }

    private void updateTelemetry() {
        telemetry.addData("Auto Stage", pathStateOne.toString());
        telemetry.addData("Shooter Velocity", Shooter.rightShooter.getVelocity());
        telemetry.addData("Path Timer", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("OpMode Timer", opModeTimer.getElapsedTimeSeconds());
        telemetry.update();
    }




    /*

     */
}