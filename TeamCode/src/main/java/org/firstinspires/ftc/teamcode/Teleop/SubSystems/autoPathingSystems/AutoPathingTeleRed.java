package org.firstinspires.ftc.teamcode.Teleop.SubSystems.autoPathingSystems;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Autonomous.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Teleop.OpModes.TestingMainOneCon;
import org.firstinspires.ftc.teamcode.Teleop.SubSystems.MovementSystems.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.Teleop.SubSystems.MovementSystems.Movement;

public class AutoPathingTeleRed {
    private Follower follower;
    private FieldCentricDrive fieldCentricDrive;
    private final Pose startingPose = new Pose(127.073732718894, 109.92626728110596, Math.toRadians(0));
    private boolean automatedDrive;

    //------- Lever Path --------- //
    private final Pose hitLeverPose = new Pose(128, 70.57972350230413, Math.toRadians(90));

    // ----- Shoot Path -------- //
    private final Pose shootPose = new Pose(102.230, 98.032, Math.toRadians(36));

    //------- Parking Path ---------- //
    private final Pose parkPose = new Pose(37.26829268292684, 33.521951219512204, Math.toRadians(180));

    private Telemetry telemetry;
    private Gamepad gamepad1;
    private boolean fieldCentric = false;

    private PathChain newPathLine(Pose start, Pose end){
        return follower.pathBuilder()
                .addPath(new BezierLine(start, end))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .build();
    }

    public void drive(){

        // ---- START AUTO PATHS ----
        if (!automatedDrive) {

            if (gamepad1.dpadLeftWasPressed()) {
                follower.followPath(newPathLine(follower.getPose(), hitLeverPose));
                automatedDrive = true;
            }
            else if (gamepad1.dpadUpWasPressed()) {
                follower.followPath(newPathLine(follower.getPose(), shootPose));
                automatedDrive = true;
            }
            else if (gamepad1.dpadRightWasPressed()) {
                follower.followPath(newPathLine(follower.getPose(), parkPose));
                automatedDrive = true;
            }
        }

        // ---- CANCEL AUTO ----
        if (automatedDrive && gamepad1.dpadDownWasPressed()) {
            follower.breakFollowing();
            automatedDrive = false;
        }
        else if (automatedDrive && !follower.isBusy()) {
            automatedDrive = false;
        }

        // ---- MANUAL DRIVE ----
        if (!automatedDrive) {
            if (fieldCentric) {
                fieldCentricDrive.drive(gamepad1);
            } else {
                Movement.drive(
                        gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x
                );
            }
        }
    }


    public void init(OpMode OP) {
        Movement.init(OP.hardwareMap);
        telemetry = OP.telemetry;
        follower = Constants.createFollower(OP.hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();

        gamepad1 = OP.gamepad1;
        fieldCentricDrive = new FieldCentricDrive(follower);
    }

    public void start() {
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive(true);
    }

    public void loop() {
        follower.update();
        drive();

        telemetry.addData("position", follower.getPose());
        telemetry.addData("velocity", follower.getVelocity());
        telemetry.addData("automatedDrive", automatedDrive);
    }
}

