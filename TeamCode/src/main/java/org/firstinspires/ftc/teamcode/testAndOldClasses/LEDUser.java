package org.firstinspires.ftc.teamcode.testAndOldClasses;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "LEDTest")
@Disabled
public class LEDUser extends OpMode{

    OpMode op;
    Servo rgb;

    @Override
    public void init() {
        op = this;
        rgb = op.hardwareMap.get(Servo.class, "LED");
        rgb.setPosition(.5);
    }

    @Override
    public void loop(){
    }


}
