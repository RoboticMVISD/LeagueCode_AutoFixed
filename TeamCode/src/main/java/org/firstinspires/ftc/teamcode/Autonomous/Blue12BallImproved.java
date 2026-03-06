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
import org.firstinspires.ftc.teamcode.testAndOldClasses.Movement;
import org.firstinspires.ftc.teamcode.Teleop.SubSystems.Shooter;
import org.firstinspires.ftc.teamcode.Autonomous.pedroPathing.Constants;


//TIMES:
@Autonomous (name = "BlueAuto12")
public class Blue12BallImproved extends OpMode{

    /*
    How this auto will work code wise is that it will have three separate methods which hold case statements.
    This is to symbolize the stages of the auto and what each stage completes/does.
     */
    private Follower follower;
    public static Servo turretRotator;



    //Variables & Enum For Shooting/Getting Preload and First Row of Balls
    private enum PathStateOne {
        DRIVE_GETTING_INTO_SHOOT_POS,
        SHOOT_PRELOAD,
        DRIVE_1ST_ROW_POS,
        HIT_LEVER,
        SET_UP_LEVER_HIT,
        DRIVE_RESET_MID_ONE,
        SHOOT_FIRST_ROW,
        DRIVE_2ND_ROW_POS,
        INTAKE_2ND_ROW,
        DRIVE_BACK_ROW_TWO,
        DRIVE_RESET_MID_TWO,
        SHOOT_SECOND_ROW,
        DRIVE_3RD_ROW_POS,
        DRIVE_RESET_MID_THREE,
        INTAKE_THIRD_ROW,
        SHOOT_THIRD_ROW,
        DRIVE_PARK;

    }
    private PathStateOne pathStateOne;
    private final Pose startPose = new Pose(32.074, 109.92626728110596, Math.toRadians(180));
    private final Pose shootPose = new Pose(55, 98, Math.toRadians(140));
    private final Pose shootPoseEX = new Pose(47.5, 98.032, Math.toRadians(140));
    private final Pose preHitLever = new Pose(45.5, 70.57972350230413, Math.toRadians(180));
    private final Pose hitLeverPose = new Pose(29.5, 70.57972350230413, Math.toRadians(180));
    private final Pose rowOneStart = new Pose(58, 80.57972350230413, Math.toRadians(180));
    private final Pose rowOneEnd = new Pose(30.5, 80.57972350230413  , Math.toRadians(180));
    private PathChain shootFirstThree, getIntoRowOnePos, getFirstRow, resetBackOne, hitLever, readyToHitLever;



    //Variables To Shoot Second Row
    private final Pose rowTwoStart = new Pose(58, 56.866359447004605, Math.toRadians(180));
    private final Pose rowTwoEnd = new Pose(20.954, 56.39631336405527, Math.toRadians(180));
    private  PathChain getIntoRowTwo, getRowTwo, resetBackTwo;




    //Variables to shoot 3rd Row and Park

    private final Pose rowThreeStart = new Pose(58, 33.5529953917, Math.toRadians(180));
    private final Pose rowThreeEnd = new Pose(20.954, 33.5529953917, Math.toRadians(180));
    private final Pose backUpSpot = new Pose(17.952, 33.5529953917, Math.toRadians(180));
    private final Pose parkPose = new Pose(45.5, 70.57972350230413, Math.toRadians(180));
    private PathChain getIntoRowThree, getRowThree, backUpRowTwo,resetBackThree, park;

    private Timer pathTimer, opModeTimer;



    private PathChain newPathLine(Pose start, Pose end){
        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(start, end))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .build();
        return path;
    }

    private PathChain newPathLine(Pose start, Pose end, Boolean isTangential){
        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(start, end))
                .setTangentHeadingInterpolation()
                .build();
        return path;
    }
    private void buildPaths(){
        //For Preload and Row One
        shootFirstThree = newPathLine(startPose, shootPose);
        getIntoRowOnePos = newPathLine(shootPose, rowOneStart);
        getFirstRow = newPathLine(rowOneStart, rowOneEnd);
        readyToHitLever = newPathLine(rowOneEnd, preHitLever);
        hitLever = newPathLine(preHitLever, hitLeverPose);
        resetBackOne = newPathLine(hitLeverPose, shootPoseEX);

        //For Row Two
        getIntoRowTwo = newPathLine(shootPose, rowTwoStart);
        getRowTwo = newPathLine(rowTwoStart, rowTwoEnd);
        backUpRowTwo = newPathLine(rowTwoEnd, rowTwoStart);
        resetBackTwo = newPathLine(rowTwoStart, shootPoseEX);

        //For Row Three and Park
        getIntoRowThree = newPathLine(shootPose,rowThreeStart);
        getRowThree = newPathLine(rowThreeStart, rowThreeEnd);
        resetBackThree = newPathLine(rowThreeEnd, shootPoseEX);
        park = newPathLine(shootPose, parkPose);
    }

    //Method/Switch statement for shooting preload and getting row one & shooting. WORKS
    private void autoCases(){
        switch (pathStateOne) {
            case DRIVE_GETTING_INTO_SHOOT_POS:
                Shooter.setShooterPower(Shooter.SPIN_UP_VELOCITY_MEDIUMRANGE - 50);//Works
                follower.followPath(shootFirstThree, true);
                setPathStateOne(PathStateOne.SHOOT_PRELOAD);
                break;
            case SHOOT_PRELOAD: //Works
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() < 3){
                    shootFromMedium();
                } else if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3){
                    turnOffSystems();
                    follower.followPath(getIntoRowOnePos, true);
                    setPathStateOne(PathStateOne.DRIVE_1ST_ROW_POS);
                } break;
            case DRIVE_1ST_ROW_POS: //Works
                if (!follower.isBusy()){
                    intakeBalls(pathTimer);
                    follower.followPath(getFirstRow, true);
                    setPathStateOne(PathStateOne.SET_UP_LEVER_HIT);
                } break;
            case SET_UP_LEVER_HIT:
                if (!follower.isBusy()){
                    follower.followPath(readyToHitLever, true);
                    setPathStateOne(PathStateOne.HIT_LEVER);
                } break;
            case HIT_LEVER:
                if (!follower.isBusy()){
                    follower.followPath(hitLever,.8,false);
                    setPathStateOne(PathStateOne.DRIVE_RESET_MID_ONE);
                }break;
            case DRIVE_RESET_MID_ONE: // Works
                if (!follower.isBusy()) {
                    Shooter.setShooterPower(Shooter.SPIN_UP_VELOCITY_MEDIUMRANGE - 50);//Works
                    follower.followPath(resetBackOne, true);
                    Shooter.setShooterPower(Shooter.SPIN_UP_VELOCITY_MEDIUMRANGE - 50);//Works
                    setPathStateOne(PathStateOne.SHOOT_FIRST_ROW);
                } break;
            case SHOOT_FIRST_ROW: //Works
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() < 3){
                    shootFromMediumEX();
                } else if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3){
                    turnOffSystems();
                    setPathStateOne(PathStateOne.DRIVE_2ND_ROW_POS);
                }
                break;
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
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() < 3.5){
                    shootFromMedium();
                } else if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3){
                    turnOffSystems();
                    setPathStateOne(PathStateOne.DRIVE_3RD_ROW_POS);
                } break;
            case DRIVE_3RD_ROW_POS:
                if (!follower.isBusy()){
                    turnOffSystems();
                    follower.followPath(getIntoRowThree);
                    setPathStateOne(PathStateOne.INTAKE_THIRD_ROW);
                } break;
            case INTAKE_THIRD_ROW:
                if (!follower.isBusy()){
                    intakeBalls(pathTimer);
                    follower.followPath(getRowThree, true);
                    setPathStateOne(PathStateOne.DRIVE_RESET_MID_THREE);
                } break;
            case DRIVE_RESET_MID_THREE:
                if (!follower.isBusy()){
                    Shooter.setShooterPower(Shooter.SPIN_UP_VELOCITY_MEDIUMRANGE);
                    follower.followPath(resetBackThree);
                    Shooter.setShooterPower(Shooter.SPIN_UP_VELOCITY_MEDIUMRANGE);
                    setPathStateOne(PathStateOne.SHOOT_THIRD_ROW);
                } break;
            case SHOOT_THIRD_ROW:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() < 3.5){
                    shootFromMedium();
                } else if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3){
                    turnOffSystems();
                    follower.followPath(park);
                    setPathStateOne(PathStateOne.DRIVE_PARK);
                }
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
    private void shootFromMedium() {
        double spped = Shooter.SPIN_UP_VELOCITY_MEDIUMRANGE - 50;
        Shooter.setShooterPower(spped, .6, turretRotator);

        if (Shooter.rightShooter.getVelocity() > spped - 40 && Shooter.rightShooter.getVelocity() < spped + 40){
            Intake.setBothIntakePower(1);
        }
    }

    private void shootFromMediumEX() {
        double spped = Shooter.SPIN_UP_VELOCITY_MEDIUMRANGE - 50;
        Shooter.setShooterPower(spped, .57, turretRotator);

        if (Shooter.rightShooter.getVelocity() > spped - 40 && Shooter.rightShooter.getVelocity() < spped + 40){
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

        Intake.init(this);
        Shooter.init(this, false);
        Movement.init(this.hardwareMap);
        turretRotator = this.hardwareMap.servo.get("turretRotator");
        turretRotator.setDirection(Servo.Direction.FORWARD);

        buildPaths();
        follower.setPose(startPose);

    }

    public void start(){
        opModeTimer.resetTimer();
    }

    public void loop(){
        follower.update();
        timerStages(opModeTimer);
        telemetry.addData("Path State: ", pathStateOne.toString());
        telemetry.addData("Path Time: ", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Total Time: ", opModeTimer.getElapsedTimeSeconds());
    }

    public void timerStages(Timer time){
        if (opModeTimer.getElapsedTimeSeconds() < 30){
            autoCases();
        } else {
            turnOffSystems();
        }
    }



    /*

     */
}