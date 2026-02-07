package org.firstinspires.ftc.teamcode.Teleop.SubSystems;


import java.lang.Math;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


public class MovementSystem {
    static OpMode op;
    public static DcMotor motorTopRight0, motorTopLeft1, motorBottomRight2, motorBottomLeft3;


    final double NORMAL_MOVE_SPEED = 0.65;
    final double MAX_MOVE_SPEED = .9;
    final double TURN_SPEED = 1.0;
    double maxSpeed = NORMAL_MOVE_SPEED;
    boolean prevIsSprintButtonDown = false;
    boolean isSprintEnabled = false;


    public static void init(OpMode opMode)
    {
        op = opMode;
        motorBottomLeft3 = opMode.hardwareMap.get(DcMotor.class, "backLeft");
        motorTopRight0 = opMode.hardwareMap.get(DcMotor.class, "frontRight");
        motorBottomRight2 = opMode.hardwareMap.get(DcMotor.class, "backRight");
        motorTopLeft1 = opMode.hardwareMap.get(DcMotor.class, "frontLeft");
        motorTopLeft1.setDirection(DcMotorSimple.Direction.REVERSE);
        motorTopRight0.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void loop()
    {
        HandleTurnInput();
        HandleMoveInput();
    }

    public void HandleTurnInput()
    {
        double turnDirection = op.gamepad1.right_stick_x * TURN_SPEED;
        setMotorPowerAll(turnDirection, turnDirection, -turnDirection, -turnDirection);
    }

    public void HandleMoveInput()
    {
        double moveDirX = op.gamepad1.left_stick_x;
        double moveDirY = -op.gamepad1.left_stick_y;
        double magnitude = Math.sqrt(moveDirX * moveDirX + moveDirY * moveDirY);

        //sprint mode
        HandleSprintButtonInput();
        if( magnitude <= 0.05) //nearly no movement (x = 0, y = 0 ) -> turn off sprint
        {
            isSprintEnabled = false;
            maxSpeed = NORMAL_MOVE_SPEED;
        }
        magnitude *= maxSpeed;

        SetMoveFromVector(moveDirX, moveDirY, magnitude);
    }
    public void HandleSprintButtonInput()
    {
        boolean isSprintButtonDown = op.gamepad1.left_stick_button;
        if(isSprintButtonDown != prevIsSprintButtonDown) //left stick pressed state changed
        {
            prevIsSprintButtonDown = isSprintButtonDown; //store left stick new state
            if(isSprintButtonDown)
            {
                ToggleSprintMode();
            }
        }
    }
    public void ToggleSprintMode()
    {
        isSprintEnabled = !isSprintEnabled; //toggle sprint enabled
        if(isSprintEnabled)
        {
            maxSpeed = MAX_MOVE_SPEED;
        }
        else
        {
            maxSpeed = NORMAL_MOVE_SPEED;
        }
    }

    public void SetMoveFromVector(double x, double y, double speedMultiplier)
    {
        double leftPower = y + x;
        double rightPower = y - x;

        double magnitude = Math.abs(y) + Math.abs(x);
        leftPower /= magnitude;
        rightPower /= magnitude;

        leftPower *= speedMultiplier;
        rightPower *= speedMultiplier;

        setMotorPowerAll(rightPower,leftPower,leftPower,rightPower);
    }

    public static void setMotorPowerAll(double pwr){
        motorBottomLeft3.setPower(pwr);
        motorTopLeft1.setPower(pwr);
        motorBottomRight2.setPower(pwr);
        motorTopRight0.setPower(pwr);
    }

    public static void setMotorPowerAll(double backLeftPwr, double frontLeftPwr, double backRightPower, double frontRightPower){
        motorBottomLeft3.setPower(backLeftPwr);
        motorTopLeft1.setPower(frontLeftPwr);
        motorBottomRight2.setPower(backRightPower);
        motorTopRight0.setPower(frontRightPower);
    }

    public static void setMotorPower(double pwr, DcMotor dcMotor){
        dcMotor.setPower(pwr);
    }
}
