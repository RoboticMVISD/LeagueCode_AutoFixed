package org.firstinspires.ftc.teamcode.testAndOldClasses;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp (name = "DW Test")
@Disabled
public class DeadWheelDataTest extends OpMode {

    OpMode op;
    DcMotor deadwheel;

    @Override
    public void init() {
        op = this;
        deadwheel = op.hardwareMap.dcMotor.get("backLeft");
        deadwheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        deadwheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        int ticks = deadwheel.getCurrentPosition();
        telemetry.addData("Dead Wheel Ticks: ", ticks);
        telemetry.update();
    }
}
