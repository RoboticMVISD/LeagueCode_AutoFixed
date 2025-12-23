package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class CampbellIntake {
    OpMode op;
    public static DcMotor intake, secondaryIntake;

    final double INTAKE_MOTOR_SPEED = 1;
    final double BALL_SUPRESSOR_SPEED = 0.3;
    double SECONDARY_INTAKE_SPEED = 1;


    public void init(OpMode OP){
        op = OP;

        intake = op.hardwareMap.get(DcMotor.class, "intake");

        secondaryIntake = op.hardwareMap.dcMotor.get("secondaryIntake");
    }

    public void loop(){
        campbellMoveIntakeConOne();
    }

    //Just Campbell's Preferences for intake. Sub out for normal when he uses controller
    public void campbellMoveIntakeConOne(){
        if (op.gamepad1.right_trigger > 0){
            intake.setPower(INTAKE_MOTOR_SPEED);
            secondaryIntake.setPower(INTAKE_MOTOR_SPEED);
        } else if (op.gamepad1.left_trigger > 0){
            intake.setPower(-INTAKE_MOTOR_SPEED);
            secondaryIntake.setPower(-SECONDARY_INTAKE_SPEED);
        } else if (op.gamepad1.a){
            intake.setPower(INTAKE_MOTOR_SPEED);
        } else if (op.gamepad1.x){
            intake.setPower(-INTAKE_MOTOR_SPEED);
        } else if (op.gamepad1.right_bumper){
            secondaryIntake.setPower(SECONDARY_INTAKE_SPEED);
        } else if (op.gamepad1.left_bumper){
            secondaryIntake.setPower(-SECONDARY_INTAKE_SPEED);
        }else {
            intake.setPower(0);
            secondaryIntake.setPower(0);
        }
    }


    /*public void testIntakeConTwo(){
        if (op.gamepad2.b){
            secondaryIntake.setPower(INTAKE_SPEED);
            intake.setPower(1);
        } else if (op.gamepad2.x){
            secondaryIntake.setPower(-INTAKE_SPEED);
        } else {
            secondaryIntake.setPower(0);
            intake.setPower(0);
        }
    }*/
}
