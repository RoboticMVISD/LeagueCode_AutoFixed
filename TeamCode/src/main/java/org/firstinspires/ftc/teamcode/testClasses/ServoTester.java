package org.firstinspires.ftc.teamcode.testClasses;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoTester {
    OpMode op;
    Servo testServo;
    double ADJUSTIBLE_POS = 0.5;

    public void init(OpMode OP) {
        op = OP;
        testServo = op.hardwareMap.servo.get("blocker");
        testServo.setDirection(Servo.Direction.REVERSE);
    }

    public void loop() {
        servoTester();
    }

    public void servoTester() {
        if (op.gamepad1.b) {
            testServo.setPosition(0);
        } else if (op.gamepad1.a) {
            testServo.setPosition(1);
        } else if (op.gamepad1.y){
            testServo.setPosition(ADJUSTIBLE_POS);
        }else if (op.gamepad1.dpadLeftWasPressed()){
            ADJUSTIBLE_POS -= .1;
        } else if (op.gamepad1.dpadRightWasPressed()){
            ADJUSTIBLE_POS += .1;
        }
        op.telemetry.addLine("Adjustive Pos #: " + ADJUSTIBLE_POS);
        op.telemetry.addLine("Current Pos: " + testServo.getPosition());
    }
}

