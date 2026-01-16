package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.List;

public class AutoAimJ {
    private AprilTagProcessor processor;
    private VisionPortal portal;
    private List<AprilTagDetection> detectedTags = new ArrayList<>();
    private OpMode op;

    public void init(OpMode OP){
        op = OP;

        processor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();

        VisionPortal.Builder builer = new VisionPortal.Builder();
        builer.setCamera(op.hardwareMap.get(WebcamName.class, "webCam"));
        builer.setCameraResolution(new Size(640, 480));
        builer.addProcessors(processor);
        portal = builer.build();
    }
}
