package org.firstinspires.ftc.teamcode.testClasses;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp (name = "Motor Test")
public class MotorTest extends OpMode{

    OpMode opMode;

    DcMotor motorFrontLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;
    @Override
    public void init(){
        opMode = this;
        motorFrontLeft = opMode.hardwareMap.dcMotor.get("leftShooter");
        motorFrontRight = opMode.hardwareMap.dcMotor.get("rightShooter");
    }

    @Override
    public void loop(){
        testMotors();
    }

    public void testMotors(){
       if (opMode.gamepad1.a){
           motorFrontLeft.setPower(.5);
       } else if (opMode.gamepad1.b){
           motorFrontRight.setPower(.5);
       } else {
           motorFrontRight.setPower(0);
           motorFrontLeft.setPower(0);
       }
    }

}
