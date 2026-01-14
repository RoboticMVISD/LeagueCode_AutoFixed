/*package org.firstinspires.ftc.teamcode.testClasses;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class LEDUser {

    OpMode op;
    Servo rgb;
    ElapsedTime timer;


    public void init(OpMode OP){
        op = OP;
        timer = new ElapsedTime();

        rgb = op.hardwareMap.get(Servo.class, "LED");
    }

    public void start(){
        timer.reset();
        timer.startTime();
        rgb.setPosition(.5);
    }

    public void loop(){
        rgbTimer();
    }
    public  void rgbTimer(){
        if (timer.seconds() > 60 && timer.seconds() < 100){
            rgb.setPosition(.388);
        } else if (timer.seconds()>100){
            rgb.setPosition(.277);
        }
    }
}
*/