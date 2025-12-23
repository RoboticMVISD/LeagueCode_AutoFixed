package org.firstinspires.ftc.teamcode.testClasses;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Teleop.Shooter;

public class PIDFCoefficientTester{

    OpMode op;

    public void init(OpMode opMode){
        op = opMode;
    }

    public void loop(){
        testP();
        testI();
        testD();
        testF();
    }

    public void testP(){
        if (op.gamepad1.bWasPressed()){
            Shooter.pidfCoefficients.p += 1;
        } else if (op.gamepad1.xWasPressed()){
            Shooter.pidfCoefficients.p -= 1;
        } else if (op.gamepad1.yWasPressed()){
            Shooter.pidfCoefficients.p += 10;
        } else if (op.gamepad1.aWasPressed()){
            Shooter.pidfCoefficients.p -= 10;
        }

        op.telemetry.addLine("Current P Variable Value: " + Shooter.proportional);
        op.telemetry.addLine("Current P in object value: " + Shooter.pidfCoefficients.p);
    }

    public void testI(){

    }

    public void testD(){

    }

    public void testF(){

    }
}
