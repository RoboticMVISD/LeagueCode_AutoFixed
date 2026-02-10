package org.firstinspires.ftc.teamcode.Teleop.SubSystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter {
    // --------- Making an OpMode object to use for hardware maps, gamepads, etc ------------ //
    static OpMode op;
    // -------- Objects for the hardware being used by turret & shooters //
    public static DcMotorEx leftShooter, rightShooter;
    public static CRServo turretRotatorCR;


    // ----- Variables for the controller two tester to find a optimal velocity for a certain range ------ //
    double TESTVELOCITY = 500;
    int SMALL_INCREMENT = 50;
    int LARGE_INCREMENT = 100;

    //Max Velocity range is -2500 to 2500
    //These are preset velocity values for certain ranges from the goal.
    //Found using the Controller two test method
    public static double SPIN_UP_VELOCITY_SHORTRANGE = 950;
    public static double SPIN_UP_VELOCITY_MEDIUMRANGE = 1050;
    public static double SPIN_UP_VELOCITY_LONGRANGE = 1225;
    public static double SPIN_UP_VELOCITY_XLRANGE = 1550;

    // ---- Variable Created to reverse the shooter if needed to put a ball back in the robot ----- /
    public static double BALL_REVERSE_SPEED = -500;


    // ---- PIDF Variables to optimize shooting and reduce oscillations, overshoot, undershoot, etc ---- //
    public static double proportional = 300;
    public static double integral = 0;
    public static double derivative = 0.001;
    public static double feedForward = 10;
    public static PIDFCoefficients pidfCoefficients = new PIDFCoefficients(proportional, integral, derivative, feedForward);

    // Boolean created to differentiate between whether we want to set all controls
    // To one controller to make testing easier, or have them separate for actual driving
    // practice or competition
    public static boolean testMode;


    // ---- Initializes Shooter Motors With PIDF Variables and Encoders, as well as the turret rotating CRServo ----- //
    public static void init(OpMode OP, boolean isTesting) {
        op = OP;

        testMode = isTesting;

        leftShooter = op.hardwareMap.get(DcMotorEx.class, "leftShooter");
        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        leftShooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftShooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftShooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        rightShooter = op.hardwareMap.get(DcMotorEx.class, "rightShooter");
        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        rightShooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightShooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightShooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        turretRotatorCR = op.hardwareMap.crservo.get("turretRotatorCR");

    }

    // ---- loop that checks the testing boolean to switch between Con One or Con two Controls ----- //
    public void loop() throws InterruptedException {
        if (testMode){
                shooterConOne();

        } else {
            shooterConTwo();
        }
        }

    // --- Con One Controls, meant to be used only for testing purposes ----- /
    public void shooterConOne() {
        /*
        A = Short Range Shooting
        B = Medium Range Shooting
        Y = Long Range Shooting
        LB = Turret Rotate left
        RB = Turret Rotate right
         */

        if (op.gamepad1.a){
            setShooterPower(SPIN_UP_VELOCITY_SHORTRANGE);
        } else if (op.gamepad1.b){
            setShooterPower(SPIN_UP_VELOCITY_MEDIUMRANGE);
        } else if (op.gamepad1.y){
            setShooterPower(SPIN_UP_VELOCITY_LONGRANGE);
        } else if (op.gamepad1.left_bumper){
            turretRotatorCR.setPower(-1);
        }else if (op.gamepad1.right_bumper){
            turretRotatorCR.setPower(1);
        }else {
            setShooterPower(0);
            turretRotatorCR.setPower(0);
        }
    }

    // --- Con two controllers, made for practice & Competition. Subject to change ---- /
    public void shooterConTwo() {
        /*
        A = Short Range Shooting
        B = Medium Range Shooting
        Y = Long Range Shooting
        X = Shooter goes reverse to put balls back into robot
        LT =  Turret rotates left
        RT = Turret rotates right
         */
        if (op.gamepad2.a){
           setShooterPower(SPIN_UP_VELOCITY_SHORTRANGE);
        } else if (op.gamepad2.b){
            setShooterPower(SPIN_UP_VELOCITY_MEDIUMRANGE);
        } else if (op.gamepad2.y){
            setShooterPower(SPIN_UP_VELOCITY_LONGRANGE);
        } else if (op.gamepad2.x){
            setShooterPower(BALL_REVERSE_SPEED);
        }else if (op.gamepad2.left_trigger > 0){
            turretRotatorCR.setPower(-1);
        }else if (op.gamepad2.right_trigger > 0){
            turretRotatorCR.setPower(1);
        }else {
            setShooterPower(0);
            turretRotatorCR.setPower(0);
        }

    }

    // --- Two "OverLoaded" methods created to make changing velocities for both shooters simultaneously easier --- //
    // "OverLoaded" = Two Methods with the same name and identifiers, but different parameters
    public static void setShooterPower(double Vel) {
        leftShooter.setVelocity(Vel); rightShooter.setVelocity(Vel);
    }

    public static void setShooterPower(double Vel, double shooterTurretPos, Servo turretRotator){
        leftShooter.setVelocity(Vel); rightShooter.setVelocity(Vel);
        turretRotator.setPosition(shooterTurretPos);
    }

    // ----- Method utilizing the test variables to have an adjustable velocity to find optimal velocity to hit a certain target ----- //
    public void testerConTwo(){

        // A = Set to chosen velocity
        // Dpad L/R to change the Test Velocity by a Small increment
        // Dpad U/D to change the Test Velocity by a Large Increment
        // Increments can be changed above where the variables are created.

        if (op.gamepad2.a){
            setShooterPower(TESTVELOCITY);
        } else if (op.gamepad2.dpadLeftWasPressed()){
            TESTVELOCITY -= SMALL_INCREMENT;
        } else if (op.gamepad2.dpadRightWasPressed()){
            TESTVELOCITY += SMALL_INCREMENT;
        } else if (op.gamepad2.dpadDownWasPressed()){
            TESTVELOCITY -= LARGE_INCREMENT;
        } else if (op.gamepad2.dpadUpWasPressed()){
            TESTVELOCITY += LARGE_INCREMENT;
        } else{
            setShooterPower(0);
        }

        // ----- Telemetry To report the current velocities on both sides, as well as what it SHOULD be ----- /
        op.telemetry.addLine("Spin Power Left: " + leftShooter.getVelocity() + " \nSpin Power Right: " + leftShooter.getVelocity() + "\nWhat Power Should be: " + TESTVELOCITY);

    }
}