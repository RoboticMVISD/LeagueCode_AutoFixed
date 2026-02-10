package org.firstinspires.ftc.teamcode.Teleop.SubSystems.MovementSystems;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Autonomous.pedroPathing.Constants;

public class PedroMovementTestandGeneral {
    private Follower follower;
    private final Pose startingPose = new Pose(8, 8, Math.toRadians(90));
    private boolean automatedDrive;
    private TelemetryManager telemetryM;
    private double slowModeMultiplier = 0.85;
    private double turnSpeedDamper = 0.5;
    Gamepad gamepad1;
    Gamepad gamepad2;

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
        hitLever = newPathLine(follower.getPose(), hitLeverPose);
        goToShoot = newPathLine(follower.getPose(), shootPose);
        goPark = newPathLine(follower.getPose(), parkPose);
    }
    public void teleOpPathFollower(){
        //Automated PathFollowing
        if (gamepad1.dpadLeftWasPressed()) {
            follower.followPath(hitLever);
            automatedDrive = true;
        }
        if (gamepad1.dpadUpWasPressed()) {
            follower.followPath(goToShoot);
            automatedDrive = true;
        }
        if (gamepad1.dpadRightWasPressed()) {
            follower.followPath(goPark);
            automatedDrive = true;
        }

        //Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.dpadDownWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }
    }

    public void drive(){
        if (!automatedDrive) {
            follower.setTeleOpDrive(
                    gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    -gamepad1.right_stick_x * turnSpeedDamper,
                    false // Robot Centric
            );
        }
    }

    public void init(OpMode OP) {
        follower = Constants.createFollower(OP.hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();

        gamepad1 = OP.gamepad1;
        gamepad2 = OP.gamepad2;

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

    }

    public void start() {
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive(false);
    }

    public void loop() {
        follower.update();
        telemetryM.update();
        buildPaths();
        drive();
        teleOpPathFollower();

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
    }
}

