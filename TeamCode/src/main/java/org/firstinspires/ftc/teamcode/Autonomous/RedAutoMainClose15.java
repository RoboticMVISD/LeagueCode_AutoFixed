package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Teleop.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.Teleop.SubSystems.MovementSystem;
import org.firstinspires.ftc.teamcode.Teleop.SubSystems.Shooter;
import org.firstinspires.ftc.teamcode.Autonomous.pedroPathing.Constants;


//TIMES:
@Autonomous (name = "RedAutoMain15")
public class RedAutoMainClose15 extends OpMode{

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    private int timesRowIntaken = 0;

    private enum PathStateOne {
        DRIVE_GETTING_INTO_SHOOT_POS,
        SHOOT_PRELOAD,
        DRIVE_ROW_TWO,
        INTAKE_ROW_TWO,
        DRIVE_RESET_FROM_TWO,
        SHOOT_ROW_TWO;
    }
    private PathStateOne pathStateOne;
    private final Pose startPose = new Pose(128.073732718894, 111.92626728110596, Math.toRadians(0));
    private final Pose shootPose = new Pose(102.230, 98.032, Math.toRadians(36));
    private final Pose shootPoseEX = new Pose(102.230, 98.032, Math.toRadians(45));
    private final Pose rowTwoStart = new Pose(99.76036866359446, 56.866359447004605, Math.toRadians(0));
    private final Pose rowTwoEnd = new Pose(134.04608294930875, 56.39631336405527, Math.toRadians(0));


    private PathChain shootPreload, driveRowTwo, intakeRowTwo, driveResetFromRowTwo;

    private enum PathStateTwo {
        DRIVE_PREINTAKE_RAMP,
        DRIVE_INTAKE_RAMP,
        INTAKE_RAMP_ARTIFACTS,
        DRIVE_EXIT_RAMP_INTAKE,
        DRIVE_RESET_TO_SHOOT,
        SHOOT_RAMP;
    }
    private PathStateTwo pathStateTwo;
    private final Pose preIntakeRampPose = new Pose(126,60, Math.toRadians(90));
    private final Pose intakeRampPose = new Pose(135, 57, Math.toRadians(45));
    private  PathChain drivePreIntakeRampOne, driveIntakeRampOne, driveExitRampOne, driveResetShootOne;

    private enum PathStateThree {
       DRIVE_FIRST_ROW,
        INTAKE_FIRST_ROW,
        DRIVE_RESET_BACK,
        SHOOT_FIRST_ROW,
        DRIVE_PARK;
    }
    private PathStateThree pathStateThree;

    private final Pose rowOneStart = new Pose(102.820, 80.57972350230413, Math.toRadians(0));
    private final Pose rowOneEnd = new Pose(126.50241820768138, 80.57972350230413  , Math.toRadians(0));
    private final Pose parkPose = new Pose(94.72089761570828, 63.82047685834502, Math.toRadians(270));
    private PathChain driveFirstRow, driveIntakeFirstRow, driveResetBackFromRowOne, drivePark;

    private PathChain newPathLine(Pose start, Pose end){
        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(start, end))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .build();
        return path;
    }
    private void buildPaths(){
        //For Preload and Row Two
        shootPreload = newPathLine(startPose, shootPose);
        driveRowTwo = newPathLine(shootPose, rowTwoStart);
        intakeRowTwo = newPathLine(rowTwoStart, rowTwoEnd);
        driveResetFromRowTwo = newPathLine(rowTwoEnd, shootPoseEX);

        //For Ramp Intaking
        drivePreIntakeRampOne = newPathLine(shootPoseEX, preIntakeRampPose);
        driveIntakeRampOne = newPathLine(preIntakeRampPose, intakeRampPose);
        driveExitRampOne = newPathLine(intakeRampPose, preIntakeRampPose);
        driveResetShootOne = newPathLine(preIntakeRampPose, shootPoseEX);

        //For Row One Intake,Shoot, and Park
        driveFirstRow = newPathLine(shootPoseEX, rowOneStart);
        driveIntakeFirstRow = newPathLine(rowOneStart, rowOneEnd);
        driveResetBackFromRowOne = newPathLine(rowOneEnd, shootPoseEX);
        drivePark = newPathLine(shootPoseEX, parkPose);

    }

    //Method/Switch statement for shooting preload and getting row one & shooting. WORKS
    private void pathStateUpdateOne(){
        switch (pathStateOne) {
            case DRIVE_GETTING_INTO_SHOOT_POS:
                Shooter.setShooterPower(Shooter.SPIN_UP_VELOCITY_MEDIUMRANGE - 50);//Works
                follower.followPath(shootPreload, true);
                setPathStateOne(PathStateOne.SHOOT_PRELOAD);
                break;
            case SHOOT_PRELOAD: //Works
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() < 3){
                    shootFromMedium();
                } else if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3){
                    turnOffSystems();
                    setPathStateOne(PathStateOne.DRIVE_ROW_TWO);
                } break;
            case DRIVE_ROW_TWO:
                if (!follower.isBusy()){
                    follower.followPath(driveRowTwo, true);
                    setPathStateOne(PathStateOne.INTAKE_ROW_TWO);
                } break;
            case INTAKE_ROW_TWO:
                if (!follower.isBusy()){
                    follower.followPath(intakeRowTwo, .4, true);
                    setPathStateOne(PathStateOne.DRIVE_RESET_FROM_TWO);
                } break;
            case DRIVE_RESET_FROM_TWO:
                if (!follower.isBusy()){
                    Shooter.setShooterPower(Shooter.SPIN_UP_VELOCITY_MEDIUMRANGE - 50);
                    follower.followPath(driveResetFromRowTwo, true);
                    setPathStateOne(PathStateOne.SHOOT_ROW_TWO);
                } break;
            case SHOOT_ROW_TWO:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() < 3){
                    shootFromMediumEX();
                } else if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3){
                    turnOffSystems();
                } break;
            default:
                turnOffSystems();
                break;
        }
    } // WORKS

    //Method/Case statement meant for the 2nd row of balls and shooting them
    private void pathStateUpdateTwo(){
        switch(pathStateTwo){
            case DRIVE_PREINTAKE_RAMP:
                if (!follower.isBusy()){
                    follower.followPath(drivePreIntakeRampOne, true);
                    setPathStateTwo(PathStateTwo.DRIVE_INTAKE_RAMP);
                } break;
            case DRIVE_INTAKE_RAMP:
                if (!follower.isBusy()){
                    follower.followPath(driveIntakeRampOne, .6, true);
                    setPathStateTwo(PathStateTwo.INTAKE_RAMP_ARTIFACTS);
                } break;
            case INTAKE_RAMP_ARTIFACTS:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() < 5){
                    intakeBalls(pathTimer);
                } else if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 5){
                    turnOffSystems();
                    setPathStateTwo(PathStateTwo.DRIVE_EXIT_RAMP_INTAKE);
                } break;
            case DRIVE_EXIT_RAMP_INTAKE:
                if (!follower.isBusy()){
                    follower.followPath(driveExitRampOne, true);
                    setPathStateTwo(PathStateTwo.DRIVE_EXIT_RAMP_INTAKE);
                } break;
            case DRIVE_RESET_TO_SHOOT:
                if (!follower.isBusy()){
                    Shooter.setShooterPower(Shooter.SPIN_UP_VELOCITY_MEDIUMRANGE - 50);
                    follower.followPath(driveResetShootOne, true);
                    setPathStateTwo(PathStateTwo.SHOOT_RAMP);
                } break;
            case SHOOT_RAMP:
                    if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() < 3){
                        shootFromMedium();
                    } else if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3 && timesRowIntaken < 2){
                        turnOffSystems();
                        timesRowIntaken++;
                        setPathStateTwo(PathStateTwo.DRIVE_PREINTAKE_RAMP);
                    } else if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3 && timesRowIntaken == 2){
                        turnOffSystems();
                        timesRowIntaken++;
                    } break;
            default:
                turnOffSystems();
                break;
        }
    } // WORKS

    public void pathStateUpdateThree(){
        switch(pathStateThree){
            case DRIVE_FIRST_ROW:
                if (!follower.isBusy()){
                    follower.followPath(driveFirstRow, true);
                    setPathStateThree(PathStateThree.INTAKE_FIRST_ROW);
                } break;
            case INTAKE_FIRST_ROW:
                if (!follower.isBusy()){
                    intakeBalls(pathTimer);
                    follower.followPath(driveIntakeFirstRow, .4, true);
                    setPathStateThree(PathStateThree.DRIVE_RESET_BACK);
                } break;
            case DRIVE_RESET_BACK:
                if (!follower.isBusy()){
                    Shooter.setShooterPower(Shooter.SPIN_UP_VELOCITY_MEDIUMRANGE - 50);
                    follower.followPath(driveResetBackFromRowOne, true);
                    setPathStateThree(PathStateThree.SHOOT_FIRST_ROW);
                } break;
            case SHOOT_FIRST_ROW:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() < 3){
                    shootFromMedium();
                } else if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3){
                    turnOffSystems();
                    setPathStateThree(PathStateThree.DRIVE_PARK);
                } break;
            case DRIVE_PARK:
                if (!follower.isBusy()){
                    follower.followPath(drivePark);
                    break;
                } break;
            default:
                turnOffSystems();
                break;
        }
    }



    private void setPathStateOne(PathStateOne newState) {
        pathStateOne = newState;
        pathTimer.resetTimer();
    }
    private void setPathStateTwo(PathStateTwo newState) {
        pathStateTwo = newState;
        pathTimer.resetTimer();
    }
    private void setPathStateThree(PathStateThree newState) {
        pathStateThree = newState;
        pathTimer.resetTimer();
    }

    private void shootFromMedium() {
        double spped = Shooter.SPIN_UP_VELOCITY_MEDIUMRANGE - 50;
        Shooter.setShooterPower(spped, .25);

        if (Shooter.rightShooter.getVelocity() > spped - 40 && Shooter.rightShooter.getVelocity() < spped + 40){
            Intake.setBothIntakePower(1);
        }
    }
    private void shootFromMediumEX() {
        double spped = Shooter.SPIN_UP_VELOCITY_MEDIUMRANGE - 50;
        Shooter.setShooterPower(spped, .15);

        if (Shooter.rightShooter.getVelocity() > spped - 40 && Shooter.rightShooter.getVelocity() < spped + 40){
            Intake.setBothIntakePower(1);
        }
    }
    private void turnOffSystems() {
        Shooter.setShooterPower(0);
        Intake.setBothIntakePower(0);
    }
    public void intakeBalls(Timer time){
        Intake.intake.setPower(1);
    }


    public void init() {
        pathStateOne = PathStateOne.DRIVE_GETTING_INTO_SHOOT_POS;
        pathStateTwo = PathStateTwo.DRIVE_PREINTAKE_RAMP;
        pathStateThree = PathStateThree.DRIVE_FIRST_ROW;


        pathTimer = new Timer();
        opModeTimer = new Timer();
        timesRowIntaken = 0;


        follower = Constants.createFollower(hardwareMap);


        Intake.init(this);
        Shooter.init(this);
        MovementSystem.init(this);

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
        if (opModeTimer.getElapsedTimeSeconds() < 11){
            pathStateUpdateOne();
        } else if (opModeTimer.getElapsedTimeSeconds() > 11){
            if (timesRowIntaken < 2){
                pathStateUpdateTwo();
            } else {
                pathStateUpdateThree();
            }
        } else {
            turnOffSystems();
        }
    }



    /*

     */
}