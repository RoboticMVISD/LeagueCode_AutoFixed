package org.firstinspires.ftc.teamcode.testClasses;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class VelocityTest{

    DcMotorEx leftShooter, rightShooter;
    OpMode opMode;

    private double leftPowerMin = 0;
    private double leftPowerMax = 1;

    private double rightPowerMin = 0;
    private double rightPowerMax = 1;

    public void init(OpMode OP){
        opMode = OP;

        leftShooter = opMode.hardwareMap.get(DcMotorEx.class, "leftShooter");
        leftShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightShooter = opMode.hardwareMap.get(DcMotorEx.class, "rightShooter");
        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void loop(){
        motorTelemetry();
        motorVelTest();
    }

    public void motorTelemetry(){
        opMode.telemetry.addLine("Left Motor Velocity: " + leftShooter.getVelocity());
        opMode.telemetry.addLine("Right Motor Velocity: " + rightShooter.getVelocity());
    }

    public void motorVelTest(){
        if (opMode.gamepad1.b){
            leftShooter.setPower(leftPowerMax);
        } else if(opMode.gamepad1.a){
            leftShooter.setPower(-leftPowerMax);
        }else if (opMode.gamepad1.x){
            rightShooter.setPower(rightPowerMax);
        } else if(opMode.gamepad1.y){
            rightShooter.setPower(-rightPowerMax);
        }else {
            leftShooter.setPower(leftPowerMin);
            rightShooter.setPower(rightPowerMin);
        }
    }
}
