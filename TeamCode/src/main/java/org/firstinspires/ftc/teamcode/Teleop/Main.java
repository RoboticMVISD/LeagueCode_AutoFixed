package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.teamcode.AutoAim;


@TeleOp (name = "Teleop 2025")
public class Main extends OpMode {

   MovementSystem movementSystem = new MovementSystem();
   Intake intake = new Intake();
   Shooter shooter = new Shooter();


   @Override
   public void init()
    {
        intake.init(this);
        movementSystem.init(this);
        shooter.init(this);
        AutoAim.init(this);
    }

    @Override
    public void loop()
    {
        intake.loop();
        movementSystem.loop();
        AutoAim.loop();

        try {
            shooter.loop();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        telemetry.addData("Is AutoAim on? ", AutoAim.aimEnabled);
        telemetry.addData("Is AutoDistancing on?", AutoAim.launcherRequested);
    }
}
