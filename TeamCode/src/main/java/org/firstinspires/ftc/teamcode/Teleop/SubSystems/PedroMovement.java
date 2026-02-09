package org.firstinspires.ftc.teamcode.Teleop.SubSystems;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Autonomous.pedroPathing.Constants;

import java.util.function.Supplier;

public class PedroMovement {
    private Follower follower;
    public static Pose startingPose;
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    OpMode op;
    Gamepad gamepad1;


    public void init(OpMode OP) {
        Movement.init(op.hardwareMap);
        follower = Constants.createFollower(op.hardwareMap);
        follower.drivetrain.setMaxPowerScaling(.5);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        startingPose.setHeading(0);
        follower.update();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();

        op = OP;

        gamepad1 = op.gamepad1;
    }

    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive(true);
    }

    public void loop() {
        //Call this once per loop
        follower.update();
        telemetryM.update();

        drivePedro();
    }

    public void drivePedro(){
        if (!automatedDrive) {
            if (!slowMode) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y ,
                    -gamepad1.left_stick_x ,
                    -gamepad1.right_stick_x ,
                    false // Robot Centric
            );

            //Automated PathFollowing
            if (gamepad1.aWasPressed()) {
                follower.followPath(pathChain.get());
                automatedDrive = true;
            }

            //Stop automated following if the follower is done
            if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
                follower.startTeleopDrive();
                automatedDrive = false;
            }
        }
    }
}
