package org.firstinspires.ftc.teamcode.testAndOldClasses;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Motor Drive test")
public class driveTest extends OpMode {
    private DcMotor backLeft;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backRight;

    @Override
    public void init(){
        backLeft = hardwareMap.dcMotor.get("backLeft");
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backRight = hardwareMap.dcMotor.get("backRight");

        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void loop(){
        if (gamepad1.a){
            backLeft.setPower(.5);
        } else if (gamepad1.b){
            frontLeft.setPower(.5);
        } else if (gamepad1.x){
            backRight.setPower(.5);
        } else if (gamepad1.y){
            frontRight.setPower(.5);
        } else {
            frontRight.setPower(0);
            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontLeft.setPower(0);
        }

        telemetry.addLine("A = BackLeft B = Front Left");
        telemetry.addLine("X = Back Right Y = Front Right");
    }

}


