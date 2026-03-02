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


    private static double kP = 0.0185;
    private static double kD = 0.0001;
    private static double goalX;
    private static double currentGoalBearing;
    private static double lastError;
    private static final double aimBoundary = 1;
    private static final double lockBoundary = 3;
    private static final double maxTurretPower = 1;
    private static double turretPower = 0;
    public static boolean aimEnabled;
    public static boolean launcherRequested;
    private static int currentGoalTag;
    private static double currentGoalElevation;
    private static double currentGoalRange;
    private static boolean noTagDetected;
    private static boolean refindTargetAttempted = false;
    private static boolean targetDataIsStale;
    static final private double launchMultiplier = 2.5;
    static final private double launchOffset = 756.68282;
    private static double targetLaunchVelocity;
    static double[] stepSizes = {0.1, 0.01, 0.001, 0.0001, 0.00001};
    static int stepIndex = 0;
    private static Servo indicatorLight;
    private static int ticks;
    private static Boolean goingToOtherBoundary, atRightBound, atLeftBound;

    private static final ElapsedTime timer = new ElapsedTime();

    public void init(OpMode OP) {
        op = OP;

        limelight = op.hardwareMap.get(Limelight3A.class, "limelight");

        indicatorLight = op.hardwareMap.get(Servo.class, "LED");

        turretTracker = op.hardwareMap.dcMotor.get("backLeft");
        turretTracker.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretTracker.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        limelight.pipelineSwitch(0);

        aimEnabled = false;
        launcherRequested = false;
        currentGoalTag = -1;
        /*
         * Starts polling for data.
         */
        limelight.start();
    }

    public void loop(boolean isOneCon) {
        limelightDataAndTelemetry();

        //autoDistanceLauncher();

        if (isOneCon){
            conOneAutoAimAndDistanceControls();
        } else {
            conTwoAutoAimAndDistanceControls();
        }

        indicatorController();

        if (launcherRequested) {
            autoDistanceLauncher();
        } else {
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
        timer.reset();
    }

    public static void update() throws InterruptedException {
        double deltaTime = timer.seconds();
        timer.reset();

        if (!aimEnabled) {
            Shooter.setRotatorPower(0);
            lastError = 0;
            return;
        }

        if (noTagDetected) {
            if (atRightBound && lastError < 0) {
                if (!goingToOtherBoundary) {
                    goToOtherBoundary();
                }
                return;
            } else if (atLeftBound && lastError > 0) {
                if (!goingToOtherBoundary) {
                    goToOtherBoundary();
                }
                return;
            }

            Shooter.setRotatorPower(0);
            lastError = 0;

            if (!refindTargetAttempted && targetDataIsStale) {
                if (currentGoalElevation > 0) {
                    Shooter.setRotatorPower(maxTurretPower);
                } else {
                    Shooter.setRotatorPower(-maxTurretPower);
                }
                Thread.sleep(100);

                Shooter.setRotatorPower(0);

                refindTargetAttempted = true;
            }

            searchFunction();

            return;
        }

        double error = currentGoalBearing;
        double pTerm = error * kP;
        refindTargetAttempted = false;

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

                    noTagDetected = false;
                    targetDataIsStale = false;
                }
            }
        } else {
            noTagDetected = true;
        }

        targetDataIsStale = result.getStaleness() > 250;

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
        }
    }

    public static void conOneAutoAimAndDistanceControls() {
        if (op.gamepad1.rightStickButtonWasPressed()){
            launcherRequested = !launcherRequested;
        } else if (op.gamepad1.leftStickButtonWasPressed()) {
            aimEnabled = !aimEnabled;
        }
    }

    private static void setLauncherPower() {
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

        if (ticks < -4800) {
            atRightBound = true;
            atLeftBound = false;
        }
        if (ticks > 8700) {
            atLeftBound = true;
            atRightBound = false;
        }

        op.telemetry.addData("Current Turret Pos: ", ticks);
    }

    private static void goToOtherBoundary() {
        goingToOtherBoundary = true;

        if (ticks > 0) {
            while (ticks < 8700 && noTagDetected) {
                Shooter.setRotatorPower(0.8);
            }
            Shooter.setRotatorPower(0);
        } else {
            while (ticks > -4800 && noTagDetected) {
                Shooter.setRotatorPower(-0.8);
            }
            Shooter.setRotatorPower(0);
        }

        goingToOtherBoundary = false;
    }

    private static void searchFunction() {
        if (ticks > 3000) {
            while (ticks > 3000) {
                Shooter.setRotatorPower(-1);
            }
        } else if (ticks < -3000) {
            while (ticks < -3000) {
                Shooter.setRotatorPower(1);
            }
        }

        while (aimEnabled && noTagDetected) {
            if (ticks > 0) {
                while (ticks > -3000) {
                    Shooter.setRotatorPower(0.5);
                }
            } else {
                while (ticks < 3000) {
                    Shooter.setRotatorPower(-0.5);
                }
            }
        }
    }
}