package org.firstinspires.ftc.teamcode.Teleop.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.teamcode.Teleop.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.testAndOldClasses.Movement;
import org.firstinspires.ftc.teamcode.Teleop.SubSystems.PedroMovement;
import org.firstinspires.ftc.teamcode.Teleop.SubSystems.Shooter;


@TeleOp (name = "Teleop 2025")
public class Main extends OpMode {

    Intake intake = new Intake();
    Shooter shooter = new Shooter();
    Movement movement = new Movement();
    PedroMovement pedroMovement = new PedroMovement();

    @Override
    public void init()
    {
        intake.init(this);
        Shooter.init(this, false);
        pedroMovement.init(this);
    }

    @Override
    public void start(){
        pedroMovement.start();
    }

    @Override
    public void loop(){
        intake.loop();
        pedroMovement.loop();

        try {
            shooter.loop();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }


}