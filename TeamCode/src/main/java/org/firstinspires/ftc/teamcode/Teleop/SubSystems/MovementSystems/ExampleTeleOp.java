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

import org.firstinspires.ftc.teamcode.Autonomous.pedroPathing.Constants;

import java.util.function.Supplier;

@Configurable
@TeleOp
public class ExampleTeleOp extends OpMode {
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = true;
    private double slowModeMultiplier = 0.8;
    private double turnSpeedMultiplier = 0.5;

    //------- Lever Path --------- //
    private final Pose hitLeverPose = new Pose(128, 70.57972350230413, Math.toRadians(90));
    private PathChain hitLever;

    // ----- Shoot Path -------- //
    private final Pose shootPose = new Pose(102.230, 98.032, Math.toRadians(36));
    private PathChain goToShoot;

    //------- Parking Path ---------- //
    private final Pose parkPose = new Pose(37.26829268292684, 33.521951219512204, Math.toRadians(180));
    private PathChain goPark;

    private PathChain newPathLine(Pose start, Pose end){
        return follower.pathBuilder()
                .addPath(new BezierLine(start, end))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .build();
    }

    public void buildPaths(){
        follower.update();
        hitLever = newPathLine(follower.getPose(), hitLeverPose);
        goToShoot = newPathLine(follower.getPose(), shootPose);
        goPark = newPathLine(follower.getPose(), parkPose);
    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose(8,8, Math.toRadians(0)) : startingPose);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        buildPaths();
    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        //Call this once per loop
        follower.update();
        telemetryM.update();

        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors

            //This is the normal version to use in the TeleOp
            if (!slowMode) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x,
                    true // Robot Centric
            );
                //This is how it looks with slowMode on
            else follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    gamepad1.left_stick_x * slowModeMultiplier,
                    gamepad1.right_stick_x * turnSpeedMultiplier,
                    true // Robot Centric
            );
        }

        //Automated PathFollowing
        if (gamepad1.dpadLeftWasPressed()) {
            follower.followPath(hitLever);
            automatedDrive = true;
        }

        //Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.dpadDownWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
    }
}