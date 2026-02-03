package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.teamcode.AutoAimJ;
import org.firstinspires.ftc.teamcode.testClasses.PositionalAutoAim;
import org.firstinspires.ftc.teamcode.testClasses.PositionalShooter;


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
        Shooter.init(this);
    }

    @Override
    public void loop(){
        intake.loop();
        movementSystem.loop();

        try {
            shooter.loop();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }


}