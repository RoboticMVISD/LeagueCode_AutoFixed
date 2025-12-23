package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcontroller.external.samples.UtilityOctoQuadConfigMenu;
import org.firstinspires.ftc.teamcode.Teleop.Intake;
import org.firstinspires.ftc.teamcode.Teleop.MovementSystem;
import org.firstinspires.ftc.teamcode.Teleop.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.util.Timer;
@Autonomous (name = "PedroAuto")
public class PedroAuto extends OpMode {

    Follower follower;
    OpMode op;

    enum PathState {
        GETTING_INTO_SHOOT_POS,
        SPIN_UP,
        SHOOTING;
    }

    PathState pathState;

    Timer pathTimer, opModeTimer;
    private final Pose startPose = new Pose(123.72119487908962, 122.9018492176387, Math.toRadians(45));
    private final Pose shootPose = new Pose(99.34566145092461, 98.73115220483643, Math.toRadians(45));

    private PathChain path;

    public void buildPaths(){
        path = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
    }

    public void pathStateUpdate(){
        switch (pathState) {
            case GETTING_INTO_SHOOT_POS:
                    follower.followPath(path, true);
                    setPathState(PathState.SHOOTING);
                break;
            case SHOOTING:
               if (!follower.isBusy()){
                   Shooter.setShooterPower(Shooter.SPIN_UP_VELOCITY_MEDIUMRANGE);
                   if (Shooter.rightShooter.getVelocity() >= Shooter.SPIN_UP_VELOCITY_MEDIUMRANGE - 20 && Shooter.rightShooter.getVelocity() <= Shooter.SPIN_UP_VELOCITY_MEDIUMRANGE + 20){
                       Intake.setBothIntakePower(1);
                   }
               }
            default:
                telemetry.addLine("No State Commanded");
                break;

        }
    }

    public void setPathState(PathState newState){
        pathState = newState;
        pathTimer.resetTimer();
    }



    @Override
    public void init() {
        pathState = PathState.GETTING_INTO_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        Intake.init(this);
        Shooter.init(this);
        MovementSystem.init(this);

        buildPaths();
        follower.setPose(startPose);
    }

    @Override
    public void loop(){
        follower.update();
        pathStateUpdate();

        telemetry.addData("Path State: ", pathState.toString());
        telemetry.addData("Path Time: ", pathTimer.getElapsedTime());
}


}
