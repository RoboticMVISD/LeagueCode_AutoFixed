package org.firstinspires.ftc.teamcode.Teleop.SubSystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class IntakeMila extends OpMode {

    public DcMotor intake;


    @Override
    public void init() {
        intake= this.hardwareMap.dcMotor.get("intake");

    }

    @Override
    public void loop() {
        if (this.gamepad1.a) {
            intake.setPower(1);
        } else if (this.gamepad1.b) {
            intake.setPower(-1);
        } else {
            intake.setPower(0);
        }

    }
}
