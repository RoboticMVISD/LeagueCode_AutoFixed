package org.firstinspires.ftc.teamcode.Teleop.SubSystems.MovementSystems;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Autonomous.pedroPathing.Constants;

import java.util.function.Supplier;

@Configurable
public class RedMove {
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    //------- Lever Path --------- //
    private final Pose hitLeverPose = new Pose(128, 70.57972350230413, Math.toRadians(90));

    // ----- Shoot Path -------- //
    private final Pose shootPose = new Pose(102.230, 98.032, Math.toRadians(36));

    //------- Parking Path ---------- //
    private final Pose parkPose = new Pose(37.26829268292684, 33.521951219512204, Math.toRadians(180));
    private boolean slowMode = false;
    private double turnMulti = 0.6;
    private double moveMulti = 1.5;
    private double strafeMulti = 1;
    Gamepad gamepad1;

    public void init(OpMode op) {
        follower = Constants.createFollower(op.hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose(0,0,Math.toRadians(0)) : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        gamepad1 = op.gamepad1;

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();
    }

    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
    }

    public void loop() {
        //Call this once per loop
        follower.update();
        telemetryM.update();
        drive();
        autoPathing();



        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
    }
    public void drive(){
        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors

            //This is the normal version to use in the TeleOp
            if (!slowMode) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * moveMulti,
                    -gamepad1.left_stick_x * strafeMulti,
                    -gamepad1.right_stick_x * turnMulti,
                    false // Robot Centric
            );
        }
    }
    private PathChain newPathLine(Pose end){
        return follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, end)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, end.getHeading(), .8))
                .build();
    }

    public void autoPathing(){
        //Automated PathFollowing
        if (gamepad1.dpadLeftWasPressed()) {
            follower.followPath(newPathLine(hitLeverPose));
            automatedDrive = true;
        }
        if (gamepad1.dpadRightWasPressed()) {
            follower.followPath(newPathLine(parkPose));
            automatedDrive = true;
        }
        if (gamepad1.dpadUpWasPressed()) {
            follower.followPath(newPathLine(shootPose));
            automatedDrive = true;
        }


        //Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.dpadDownWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }
    }


}