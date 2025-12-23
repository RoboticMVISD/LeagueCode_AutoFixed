/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Autonomous;

import android.util.Size;
import java.lang.Math;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


import java.util.List;

/*
 * This OpMode illustrates the basics of AprilTag recognition and pose estimation,
 * including Java Builder structures for specifying Vision parameters.
 *
 * For an introduction to AprilTags, see the FTC-DOCS link below:
 * https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_intro/apriltag-intro.html
 *
 * In this sample, any visible tag ID will be detected and displayed, but only tags that are included in the default
 * "TagLibrary" will have their position and orientation information displayed.  This default TagLibrary contains
 * the current Season's AprilTags and a small set of "test Tags" in the high number range.
 *
 * When an AprilTag in the TagLibrary is detected, the SDK provides location and orientation of the tag, relative to the camera.
 * This information is provided in the "ftcPose" member of the returned "detection", and is explained in the ftc-docs page linked below.
 * https://ftc-docs.firstinspires.org/apriltag-detection-values
 *
 * To experiment with using AprilTags to navigate, try out these two driving samples:
 * RobotAutoDriveToAprilTagOmni and RobotAutoDriveToAprilTagTank
 *
 * There are many "default" VisionPortal and AprilTag configuration parameters that may be overridden if desired.
 * These default parameters are shown as comments in the code below.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@TeleOp(name = "AutoAim")

public class LogitecAutoAim extends OpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    static OpMode op;

    /**
     * The variables to store our instance of the AprilTag processor and its builder.
     */
    private static AprilTagProcessor sigmaAprilTag;

    /**
     * The variables to store our instance of the vision portal and its builder.
     */
    private static VisionPortal sigmaVisionPortal;

    /**
     * The variables to store the recognized AprilTags.
     */
    private int currentObeliskTag, currentGoalTag;
    private static double rotateDuration;
    private static double currentGoalBearing;
    private double currentGoalRange;
    private double targetLaunchVelocity;
    private static boolean noTagDetected;
    private static boolean targetLocked = false;
    public static boolean aimEnabled = false;
    public static boolean launcherRequested = false;

    static final private double turnMultiplier = 2.5;
    final private double launchMultiplier = 5.3;
    final private double launchOffset = 813.31467;
    static final private double closeBoundary = 1;
    static final private double turnPower = 1;

    public static double proportional = 300;
    public static double integral = 0;
    public static double derivative = 0.001;
    public static double feedForward = 10;
    public static PIDFCoefficients pidfCoefficients = new PIDFCoefficients(proportional, integral, derivative, feedForward);


    /**
     * The variables to store our instances of the servos.
     */
    private static CRServo bearingServo;
    private static DcMotorEx rightShooter;
    private static DcMotorEx leftShooter;

    public void init(OpMode OP) {
        op = OP;

        bearingServo = op.hardwareMap.get(CRServo.class, "turretRotator");

        leftShooter = op.hardwareMap.get(DcMotorEx.class, "leftShooter");
        leftShooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        leftShooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftShooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftShooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        rightShooter = op.hardwareMap.get(DcMotorEx.class, "rightShooter");
        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightShooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightShooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightShooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        initAprilTag();

        //Intake.init(this);

        // Wait for the DS start button to be touched.
        op.telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        op.telemetry.addData(">", "Touch START to start OpMode");
    }

    @Override
    public void init() {

    }

    public void loop() {

        telemetryAprilTag();
        op.telemetry.addData("Current Goal", currentGoalTag);
        op.telemetry.addData("Current Obelisk", currentObeliskTag);
        op.telemetry.addData("Current Bearing", currentGoalBearing);

        // Push telemetry to the Driver Station.

        if (aimEnabled) {
            try {
                positionTurret();
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        } else {
            bearingServo.setPower(0);
        }

        targetLaunchVelocity = ((launchMultiplier * currentGoalRange) + launchOffset);

        if (launcherRequested) {
            setLauncherPower();
        } else {
            rightShooter.setVelocity(0);
            leftShooter.setVelocity(0);
        }

        op.telemetry.addData("Target locked", targetLocked);

        //Intake.campbellMoveIntakeConOne();

        /* Save CPU resources; can resume streaming when needed.
                if (gamepad1.dpad_down) {
                    sigmaVisionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    sigmaVisionPortal.resumeStreaming();
                }

                // Share the CPU.
                //sleep(20);
                 */

    }   // end method runOpMode()

    /**
     * Initialize the AprilTag processor.
     */
    private static void initAprilTag() {

        // Create the AprilTag processor.
        AprilTagProcessor.Builder sigmaAprilTagBuilder = new AprilTagProcessor.Builder();

        // The following default settings are available to un-comment and edit as needed.
        sigmaAprilTagBuilder.setDrawAxes(true);
        sigmaAprilTagBuilder.setDrawCubeProjection(false);
        sigmaAprilTagBuilder.setDrawTagOutline(true);
        sigmaAprilTagBuilder.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11);
        sigmaAprilTagBuilder.setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary());
        sigmaAprilTagBuilder.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES);

        // == CAMERA CALIBRATION ==
        // If you do not manually specify calibration parameters, the SDK will attempt
        // to load a predefined calibration for your camera.
        //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
        // ... these parameters are fx, fy, cx, cy.

        sigmaAprilTag = sigmaAprilTagBuilder.build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        sigmaAprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder sigmaVisionPortalBuilder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            sigmaVisionPortalBuilder.setCamera(op.hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            sigmaVisionPortalBuilder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        sigmaVisionPortalBuilder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        sigmaVisionPortalBuilder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        sigmaVisionPortalBuilder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        sigmaVisionPortalBuilder.setAutoStopLiveView(false);

        // Set and enable the processor.
        sigmaVisionPortalBuilder.addProcessor(sigmaAprilTag);

        // Build the Vision Portal, using the above settings.
        sigmaVisionPortal = sigmaVisionPortalBuilder.build();

        // Disable or re-enable the aprilTag processor at any time.
        sigmaVisionPortal.setProcessorEnabled(sigmaAprilTag, true);

    }   // end method initAprilTag()


    /**
     * Add telemetry about AprilTag detections.
     */
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = sigmaAprilTag.getDetections();
        op.telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                op.telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                op.telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                op.telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                op.telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                if (detection.id == 21 ||detection.id == 22 ||detection.id == 23){
                    currentObeliskTag = detection.id;
                } else{
                    currentGoalTag = detection.id;
                    currentGoalBearing = detection.ftcPose.bearing;
                    currentGoalRange = detection.ftcPose.range;
                }
            } else {
                op.telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                op.telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        noTagDetected = currentDetections.isEmpty();

        // Add "key" information to telemetry
        op.telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        op.telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        op.telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()

    /**
     * using servos, position the turret to the proper trajectory for the goal.
     */
    public static void positionTurret() throws InterruptedException {

        rotateDuration = (Math.abs(currentGoalBearing) / turnPower) * turnMultiplier;

        if((currentGoalBearing > closeBoundary || currentGoalBearing < -closeBoundary) && !noTagDetected){
            if (currentGoalBearing > 0) {
                bearingServo.setPower(-turnPower);
            } else {
                bearingServo.setPower(turnPower);
            }
            Thread.sleep((long) rotateDuration);
            bearingServo.setPower(0);
        } else {
            bearingServo.setPower(0);
        }

        targetLocked = currentGoalBearing > -closeBoundary && currentGoalBearing < closeBoundary && !noTagDetected;
    }

    private void setLauncherPower() {
        leftShooter.setVelocity(targetLaunchVelocity);
        rightShooter.setVelocity(targetLaunchVelocity);

        op.telemetry.addData("Launch Speed", targetLaunchVelocity);
    }

}   // end class