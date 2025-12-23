package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.testClasses.PIDFCoefficientTester;

@TeleOp (name = "Campbell Teleop")
public class CampbellMain extends OpMode {

    CampbellMovement campbellMovement = new CampbellMovement();
    CampbellIntake campbellIntake = new CampbellIntake();
    Shooter shooter = new Shooter();
    BatteryChecker batteryChecker = new BatteryChecker();

    public void init()
    {
        campbellIntake.init(this);
        campbellMovement.init(this);
        shooter.init(this);
        batteryChecker.init(this);
    }

    @Override
    public void loop()
    {
        campbellIntake.loop();
        campbellMovement.loop();
        try {
            shooter.loop();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        batteryChecker.loop();
    }
}
