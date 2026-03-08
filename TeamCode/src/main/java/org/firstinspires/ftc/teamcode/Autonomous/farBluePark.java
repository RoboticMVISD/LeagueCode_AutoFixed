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
@Autonomous (name = "blueFarPark")
public class farBluePark extends OpMode{

    /*
    How this auto will work code wise is that it will have three separate methods which hold case statements.
    This is to symbolize the stages of the auto and what each stage completes/does.
     */
    private Follower follower;
    public static Servo turretRotator;



    //Variables & Enum For Shooting/Getting Preload and First Row of Balls
    private enum PathStateOne {
        DRIVE_GETTING_INTO_SHOOT_POS,
    }
    private PathStateOne pathStateOne;
    private final Pose startPose = new Pose(56, 7.795163584637276, Math.toRadians(90));
    private final Pose parkPose = new Pose(36.335704125177806, 8.819345661450928, Math.toRadians(90));
    private PathChain park;

    private Timer pathTimer, opModeTimer;



    private PathChain newPathLine(Pose start, Pose end){
        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(start, end))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .build();
        return path;
    }
    private void buildPaths(){
        park = newPathLine(startPose, parkPose);
    }

    //Method/Switch statement for shooting preload and getting row one & shooting. WORKS
    private void autoCases(){
        if (pathStateOne == PathStateOne.DRIVE_GETTING_INTO_SHOOT_POS) {
                follower.followPath(park, true);
        }
    }



    public void init() {
        pathStateOne = PathStateOne.DRIVE_GETTING_INTO_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);

        Movement.init(this.hardwareMap);

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
        }
    }



    /*

     */
}