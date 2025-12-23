package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.*;


@TeleOp (name = "Teleop 2025")
public class Main extends OpMode {

   MovementSystem movementSystem = new MovementSystem();
   Intake intake = new Intake();
   Shooter shooter = new Shooter();
   LEDUser ledUser = new LEDUser();
   BatteryChecker batteryChecker = new BatteryChecker();


   public void init()
    {
        intake.init(this);
        movementSystem.init(this);
        shooter.init(this);
        batteryChecker.init(this);
        ledUser.init(this);
    }

    public void start(){
       ledUser.start();
    }

    @Override
    public void loop()
    {
        intake.loop();
        movementSystem.loop();
        try {
            shooter.loop();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        batteryChecker.loop();

        ledUser.loop();
    }
}
