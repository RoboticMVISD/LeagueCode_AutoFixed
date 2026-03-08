package org.firstinspires.ftc.teamcode.Teleop.SubSystems.CameraSystems;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.tan;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Teleop.SubSystems.Shooter;

import java.util.List;

public class AutoAim_Distance {
    static OpMode op;

    private static Limelight3A limelight;
    public static DcMotor turretTracker;


    private static double kP = 0.0141;
    private static double kD = 0.0002;
    private static double goalX;
    private static double currentGoalBearing;
    private static double lastError = 0 ;
    private static final double aimBoundary = 1;
    private static final double lockBoundary = 3;
    private static final double maxTurretPower = 1;
    private static double turretPower = 0;
    public static boolean aimEnabled, searchEnabled;
    public static boolean launcherRequested;
    private static int currentGoalTag;
    private static double currentGoalElevation;
    private static double currentGoalRange;
    private static boolean noTagDetected;
    private static boolean refindTargetAttempted = false;
    private static boolean targetDataIsStale;
    static final private double launchMultiplier = 2.30;
    static final private double launchOffset = 836.68282;
    public static double targetLaunchVelocity ;
    static double[] stepSizes = {0.1, 0.01, 0.001, 0.0001, 0.00001};
    static int stepIndex = 0;
    private static Servo indicatorLight;
    private static int ticks;
    private static boolean inTeleop = false, goingToOtherBoundary = false, atRightBound = false, atLeftBound = false, atRightSearchBound = true, atLeftSearchBound = false;
    private static boolean assignedRotateDirection;

    private static final ElapsedTime aimTimer = new ElapsedTime();
    private static final ElapsedTime staleTimer = new ElapsedTime();

    public void init(OpMode OP) {
        op = OP;

        limelight = op.hardwareMap.get(Limelight3A.class, "limelight");

        indicatorLight = op.hardwareMap.get(Servo.class, "LED");

        turretTracker = op.hardwareMap.dcMotor.get("backLeft");
        turretTracker.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretTracker.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        targetLaunchVelocity = 1000;

        limelight.pipelineSwitch(0);

        aimEnabled = false;
        launcherRequested = false;
        currentGoalTag = -1;
        searchEnabled = false;
        /*
         * Starts polling for data.
         */
        limelight.start();
    }

    public void loop(boolean isOneCon, boolean isTeleop) {
        limelightDataAndTelemetry();

        inTeleop = isTeleop;

        if (isOneCon){
            conOneAutoAimAndDistanceControls();
            searchEnabled = true;
        } else {
            conTwoAutoAimAndDistanceControls();
            searchEnabled = false;
        }

        indicatorController();

        if (launcherRequested) {
            autoDistanceLauncher();
        } else if (!inTeleop){
            Shooter.setShooterPower(0);
        }

        turretTrackerTelemetry();
    }

    public static void setkP(double newKP) {
        kP = newKP;
    }

    public static void setkD(double newKD) {
        kD = newKD;
    }

    public void resetTimer() {
        aimTimer.reset();
    }

    public static void update() throws InterruptedException {
        double deltaTime = aimTimer.seconds();
        aimTimer.reset();

        if (!aimEnabled) {
            if (!inTeleop) {
                quickStopRotator();

                Shooter.setRotatorPower(0);
            }
            lastError = 0;
            return;
        }

        if (noTagDetected) {
            if (((atRightBound && lastError < 0) || (atLeftBound && lastError > 0) || goingToOtherBoundary) && !refindTargetAttempted) {
                    /*if (atRightBound) {
                        goToOtherBoundary("Right");
                    } else {
                        goToOtherBoundary("Left");
                    }
                return;*/
            }

            if (targetDataIsStale) {
                /*if (!refindTargetAttempted) {
                    if (currentGoalElevation > 0) {
                        Shooter.setRotatorPower(maxTurretPower);
                    } else {
                        Shooter.setRotatorPower(-maxTurretPower);
                    }
                    Thread.sleep(100);

                    Shooter.setRotatorPower(0);

                    refindTargetAttempted = true;

                    return;
                }*/
                if (searchEnabled) {
                    if (atLeftSearchBound) {
                        searchFunction("Left");
                    } else if (atRightSearchBound) {
                        searchFunction("Right");
                    }

                    return;
                }
            }

            quickStopRotator();

            Shooter.setRotatorPower(0);
            lastError = 0;

            return;
        }

        double error = currentGoalBearing;
        double pTerm = error * kP;
        refindTargetAttempted = false;
        goingToOtherBoundary = false;
        assignedRotateDirection = false;

        double dTerm = 0;
        if (deltaTime > 0) {
            dTerm = ((error - lastError) / deltaTime) * kD;
        }

        if (Math.abs(error) < aimBoundary) {
            turretPower = 0;
        } else {
            turretPower = Range.clip(pTerm + dTerm, -maxTurretPower, maxTurretPower);
        }

        Shooter.setRotatorPower(turretPower);
        lastError = error;
    }

    public static void limelightDataAndTelemetry(){
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                if (currentGoalTag == -1) {
                    if (fiducial.getFiducialId() == 20 || fiducial.getFiducialId() == 24) {
                        currentGoalTag = fiducial.getFiducialId();
                    }
                }

                if (fiducial.getFiducialId() == currentGoalTag) {
                    currentGoalBearing = fiducial.getTargetXDegrees();
                    currentGoalElevation = fiducial.getTargetYDegrees();
                    goalX = fiducial.getTargetXPixels();
                    double elevationInRadians = (currentGoalElevation * (PI / 180));
                    currentGoalRange = ((30 - 11.5) / (tan(0 + elevationInRadians)));

                    staleTimer.reset();
                    noTagDetected = false;
                    targetDataIsStale = false;
                }
            }
        } else {
            noTagDetected = true;
        }

        targetDataIsStale = staleTimer.milliseconds() > 1000;

        op.telemetry.addData("Tag Detected", !noTagDetected);

        op.telemetry.addData("Goal Bearing", currentGoalBearing);
        op.telemetry.addData("Goal Distance", currentGoalRange);
        op.telemetry.addData("Goal Elevation", currentGoalElevation);
    }

    public static void autoDistanceLauncher() {
        targetLaunchVelocity = ((launchMultiplier * currentGoalRange) + launchOffset);

        if (launcherRequested) {
            setLauncherPower();
        } else {
            Shooter.leftShooter.setVelocity(0);
            Shooter.rightShooter.setVelocity(0);
        }
    }

    public static void conTwoAutoAimAndDistanceControls() {
        if (op.gamepad2.dpadRightWasPressed()){
            launcherRequested = !launcherRequested;
        } else if (op.gamepad2.dpadUpWasPressed()) {
            aimEnabled = !aimEnabled;
            staleTimer.reset();
        }
    }

    public static void conOneAutoAimAndDistanceControls() {
        if (op.gamepad1.rightStickButtonWasPressed()){
            launcherRequested = !launcherRequested;
        } else if (op.gamepad1.leftStickButtonWasPressed()) {
            aimEnabled = !aimEnabled;
            staleTimer.reset();
        }
    }

    public static void setLauncherPower() {
        Shooter.leftShooter.setVelocity(targetLaunchVelocity);
        Shooter.rightShooter.setVelocity(targetLaunchVelocity);
    }

    private static void conTwoPDAdjuster() {
        if (op.gamepad2.bWasPressed()) {
            setkP(kP + stepSizes[stepIndex]);
        } else if (op.gamepad2.aWasPressed()) {
            setkP(kP - stepSizes[stepIndex]);
        } else if (op.gamepad2.yWasPressed()) {
            setkD(kD + stepSizes[stepIndex]);
        } else if (op.gamepad2.xWasPressed()) {
            setkD(kD - stepSizes[stepIndex]);
        } else if (op.gamepad2.dpadRightWasPressed()) {
            if (stepIndex < stepSizes.length - 1){
                stepIndex = stepIndex + 1;
            }
        } else if (op.gamepad2.dpadLeftWasPressed()) {
            if (stepIndex > 0) {
                stepIndex = stepIndex - 1;
            }
        }

        op.telemetry.addData("kD", kD);
        op.telemetry.addData("kP", kP);
        op.telemetry.addData("Step Size", stepSizes[stepIndex]);
    }

    private static void indicatorController() {
        if (aimEnabled & !(abs(currentGoalBearing) < lockBoundary)) {
            indicatorLight.setPosition(0.800);
        } else if (abs(currentGoalBearing) < lockBoundary) {
            indicatorLight.setPosition(0.290);
        } else if(!(Shooter.leftShooter.getVelocity() > targetLaunchVelocity - 20 && Shooter.leftShooter.getVelocity() < targetLaunchVelocity - 20)) {
            indicatorLight.setPosition(0.500);
        } else {
            indicatorLight.setPosition(0.180);
        }
    }

    //Turret Limits = -4900 (Right) && 8800 (Left)
    private static void turretTrackerTelemetry(){
        ticks = turretTracker.getCurrentPosition();

        if (ticks < -4000) {
            atRightBound = true;
            atLeftBound = false;
        }
        if (ticks > 9000) {
            atLeftBound = true;
            atRightBound = false;
        }

        if (ticks < -2500) {
            atRightSearchBound = true;
            atLeftSearchBound = false;
        }
        if (ticks > 2500) {
            atLeftSearchBound = true;
            atRightSearchBound = false;
        }

        op.telemetry.addData("Current Turret Pos: ", ticks);
    }

    private static void goToOtherBoundary(String lastBoundary) {
        goingToOtherBoundary = true;

        if (atLeftBound && assignedRotateDirection == false) {
                Shooter.setRotatorPower(0.8);
                assignedRotateDirection = true;
        } else {
                Shooter.setRotatorPower(-0.8);
                assignedRotateDirection = true;
        }

        if ((lastBoundary == "Left" && !atLeftBound) || (lastBoundary == "Right" && !atRightBound)) {
            goingToOtherBoundary = false;
            refindTargetAttempted = true;
            assignedRotateDirection = false;
        }
    }

    private static void searchFunction(String lastSearchBoundary) {
        if (lastSearchBoundary == "Left" && aimEnabled && noTagDetected) {
                Shooter.setRotatorPower(0.25);
        } else {
                Shooter.setRotatorPower(-0.25);
        }

        if (!noTagDetected) {
            quickStopRotator();
        }
    }

    private static void quickStopRotator() {
        if (Shooter.turretRotatorCR.getPower() > 0) {
            Shooter.setRotatorPower(-0.001);
        } else {
            Shooter.setRotatorPower(0.001);
        }
    }
}