package org.firstinspires.ftc.teamcode.Autonomous;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Teleop.Shooter;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class LogitechCam {

    AprilTagProcessor.Builder builder;
    AprilTagProcessor processor;
    VisionPortal portal;
    OpMode op;
    WebcamName cam;

    public void init(OpMode OP){
        op = OP;

        processor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(op.hardwareMap.get(WebcamName.class, "Webcam 1"));
        //Center of screen is 320, 240
        builder.setCameraResolution(new Size(640 , 480));
        builder.addProcessor(processor);

        portal = builder.build();
    }

    public void loop(){
        testCam();
        autoAim();
    }

    public void testCam(){
        if (op.gamepad1.dpadUpWasPressed()){
            portal.resumeStreaming();
        } else if (op.gamepad1.dpadDownWasPressed()){
            portal.stopStreaming();
        }
    }

    public void autoAim(){
        List<AprilTagDetection> currentDetections = processor.getDetections();
        op.telemetry.addData("# AprilTags Detected", currentDetections.size());

        for (AprilTagDetection detection : currentDetections) {
            double x_camera = detection.rawPose.x;
            double y_camera = detection.rawPose.y;
            double z_camera = detection.rawPose.z;

            //Center of screen is 320, 240
            if (x_camera > 1){
                Shooter.turretRotator.setPower(-.15);
            } else if (x_camera < -1){
                Shooter.turretRotator.setPower(.15);
            } else {
                Shooter.turretRotator.setPower(0);
            }

            op.telemetry.addData("X Raw Pose: ", x_camera);
            op.telemetry.addData("Y Raw Pose: ", y_camera);
            op.telemetry.addData("Z Raw Pose: ", z_camera);
            op.telemetry.addData("Robot_Center: ", detection.center);
        }

    }
}