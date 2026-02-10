package org.firstinspires.ftc.teamcode.Teleop.SubSystems.CameraSystems;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

public class WebCam {
    private AprilTagProcessor processor;
    private VisionPortal portal;
    private List<AprilTagDetection> detectedTags = new ArrayList<>();
    private Telemetry telemetry;
    private OpMode op;

    public void init(OpMode OP){
        telemetry = OP.telemetry;

        processor = new AprilTagProcessor.Builder()
                .setDrawTagID(false)
                .setDrawTagOutline(false)
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .setNumThreads(1)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(OP.hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(640,480));
        builder.enableLiveView(true);
        builder.addProcessor(processor);

        portal = builder.build();
        processor.setDecimation(3);

        setManualExposure(6, 240);
    }


    public void setManualExposure(int exposureMS, int gain){
        if (portal == null || portal.getCameraState() != VisionPortal.CameraState.STREAMING){
            telemetry.addData("Camera", "Waiting...");

            while (portal != null && portal.getCameraState() != VisionPortal.CameraState.STREAMING){
                try { Thread.sleep(20 );
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }

            telemetry.addData("Camera", "Ready!");
        }
        if (portal.getCameraState() == VisionPortal.CameraState.STREAMING){
            try {
                ExposureControl exposureControl = portal.getCameraControl(ExposureControl.class);
                GainControl gainControl = portal.getCameraControl(GainControl.class);

                if (exposureControl.getMode() != ExposureControl.Mode.Manual){
                    exposureControl.setMode(ExposureControl.Mode.Manual);
                    Thread.sleep(50);
                }

                exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
                Thread.sleep(20);
                gainControl.setGain(gain);
                Thread.sleep(20);
            } catch (Exception e) {
                telemetry.addData("Camera Control Error", e.getMessage());
            }
        }
    }

    public void update(){
        detectedTags = processor.getDetections();
    }

    public List<AprilTagDetection> getDetectedTags(){
        return detectedTags;
    }

    public AprilTagDetection getTagBySpecificId(int id){
        for (AprilTagDetection detection : detectedTags){
            if (detection.id == id){
                return detection;
            }
        }
        return null;
    }

    public void telemetryUpdate(){
        for (AprilTagDetection detection : detectedTags) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }
    }

    public void stop(){
        if (portal != null){
            portal.close();
        }
    }

}