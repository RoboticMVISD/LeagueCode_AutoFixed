package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.teamcode.Teleop.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.Teleop.SubSystems.MovementSystem;
import org.firstinspires.ftc.teamcode.Teleop.SubSystems.Shooter;


@TeleOp (name = "Teleop Testing")
public class TestingMain extends OpMode {

    MovementSystem movementSystem = new MovementSystem();
    Intake intake = new Intake();
    Shooter shooter = new Shooter();

    @Override
    public void init()
    {
        intake.init(this);
        movementSystem.init(this);
        Shooter.init(this, true);
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