package org.firstinspires.ftc.teamcode.Teleop.SubSystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Intake {
    // --------- Making an OpMode object to use for hardware maps, gamepads, etc ------------ //
    static OpMode op;
    public static DcMotor intake, secondaryIntake;

    // ---------- Speed of Both Intakes ----------- //
    final double INTAKE_MOTOR_SPEED = .8;


    // -------- Initialization for intake system. Grabs from OpMode, maps the hardware ----------- //
    public static void init(OpMode OP){
        op = OP;

        intake = op.hardwareMap.get(DcMotor.class, "intake");

        secondaryIntake = op.hardwareMap.dcMotor.get("secondaryIntake");
    }

    public void loop(){
        moveIntakeConOne();
    }


    // ----------- Method containing controls to use/move the intake ---------- //
    public void moveIntakeConOne(){

        // ------ RT = Intake In, LT = Both Intakes In, Dpad Down = Both Intakes Out -------- /

        if (op.gamepad1.right_trigger > 0){
            intake.setPower(INTAKE_MOTOR_SPEED);
        } else if (op.gamepad1.left_trigger > 0){
            setBothIntakePower(INTAKE_MOTOR_SPEED);
        } else if (op.gamepad1.dpad_down){
            setBothIntakePower(-INTAKE_MOTOR_SPEED);
        } else {
            setBothIntakePower(0);
        }
    }

    // ----- Method Created to set both intakes to a certain power to simplify code -------- //
    public static void setBothIntakePower(double pwr){
        intake.setPower(pwr);
        secondaryIntake.setPower(pwr);
    }
}
