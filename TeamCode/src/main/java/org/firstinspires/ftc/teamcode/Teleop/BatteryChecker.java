package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class BatteryChecker {

    OpMode op;
    VoltageSensor batteryVoltage;

    enum voltageState{
        GOOD,
        REPLACE;
    }

    voltageState state;
    double currentVoltage;

    public void init(OpMode OP){
        op = OP;

        //Supposedly, every motor controller has a voltage sensor hence any motor can be used
        batteryVoltage = op.hardwareMap.voltageSensor.get("Control Hub");
    }

    public void loop(){
        checkBattery();
    }

    public void checkBattery(){
        currentVoltage = batteryVoltage.getVoltage();
        if (currentVoltage > 12){
            state = voltageState.GOOD;
        } else {
            state = voltageState.REPLACE;
        }

        switch (state){
            case GOOD:
                op.telemetry.addLine("Battery = Nominal");
            case REPLACE:
                op.telemetry.addLine("Battery = Needs to be replaced");
        }
    }
}
