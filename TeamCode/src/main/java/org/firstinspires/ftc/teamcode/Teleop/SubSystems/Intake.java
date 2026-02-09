package org.firstinspires.ftc.teamcode.Teleop.SubSystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Intake {
    static OpMode op;
    public static DcMotor intake, secondaryIntake;

    final double INTAKE_MOTOR_SPEED = 1;
    double SECONDARY_INTAKE_SPEED = 1;


    public static void init(OpMode OP){
        op = OP;

        intake = op.hardwareMap.get(DcMotor.class, "intake");

        secondaryIntake = op.hardwareMap.dcMotor.get("secondaryIntake");
    }

    public void loop(){
        moveIntakeConOne();
    }

    public void moveIntakeConOne(){
        intake.setPower(op.gamepad1.right_trigger > 0 ? INTAKE_MOTOR_SPEED : 0);

        intake.setPower(op.gamepad1.left_trigger > 0 ? INTAKE_MOTOR_SPEED : 0);
        secondaryIntake.setPower(op.gamepad1.left_trigger > 0 ? INTAKE_MOTOR_SPEED : 0);

        intake.setPower(op.gamepad1.dpad_down ? -INTAKE_MOTOR_SPEED : 0);
        secondaryIntake.setPower(op.gamepad1.dpad_down ? -INTAKE_MOTOR_SPEED : 0);
    }

    public static void setBothIntakePower(double pwr){
        intake.setPower(pwr);
        secondaryIntake.setPower(pwr);
    }
}
