package org.firstinspires.ftc.teamcode.Teleop.SubSystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Intake {
    static OpMode op;
    public static DcMotor intake, secondaryIntake;

    final double INTAKE_MOTOR_SPEED = .8;


    public static void init(OpMode OP){
        op = OP;

        intake = op.hardwareMap.get(DcMotor.class, "intake");

        secondaryIntake = op.hardwareMap.dcMotor.get("secondaryIntake");
    }

    public void loop(){
        moveIntakeConOne();
    }

    public void moveIntakeConOne(){
        if (op.gamepad1.right_trigger > 0){
            intake.setPower(INTAKE_MOTOR_SPEED);
        } else if (op.gamepad1.left_trigger > 0){
            intake.setPower(INTAKE_MOTOR_SPEED);
            secondaryIntake.setPower(INTAKE_MOTOR_SPEED);
        } else if (op.gamepad1.dpad_down){
            intake.setPower(-INTAKE_MOTOR_SPEED);
            secondaryIntake.setPower(-INTAKE_MOTOR_SPEED);
        } else {
            intake.setPower(0);
            secondaryIntake.setPower(0);
        }
    }

    public static void setBothIntakePower(double pwr){
        intake.setPower(pwr);
        secondaryIntake.setPower(pwr);
    }
}
