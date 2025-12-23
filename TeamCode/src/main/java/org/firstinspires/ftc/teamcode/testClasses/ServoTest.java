package org.firstinspires.ftc.teamcode.testClasses;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoTest {

    OpMode op;
    private Servo servo = null;

    public void init(OpMode OP){
        op = OP;
        servo = op.hardwareMap.servo.get("angleChanger");
        servo.setDirection(Servo.Direction.FORWARD);
        servo.scaleRange(0,0.5);
    }

    public void loop(){
        testServoPos();
        op.telemetry.addLine("Servo Position: " + servo.getPosition());
    }

    public void testServoPos(){
        if (op.gamepad1.bWasPressed()) {
            servo.setPosition(0);
        } else if (op.gamepad1.aWasPressed()){
            servo.setPosition(.8);
        } else if (op.gamepad1.aWasReleased() || op.gamepad1.bWasReleased()){
            servo.setPosition(.3);
        }
    }
}
