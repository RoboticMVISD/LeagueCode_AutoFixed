package org.firstinspires.ftc.teamcode.Teleop.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.teamcode.Teleop.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.Teleop.SubSystems.PedroMovement;
import org.firstinspires.ftc.teamcode.Teleop.SubSystems.Shooter;


@TeleOp (name = "Teleop Testing")
public class TestingMain extends OpMode {

    PedroMovement pedroMovement = new PedroMovement();
    Intake intake = new Intake();
    Shooter shooter = new Shooter();

    @Override
    public void init()
    {
        intake.init(this);
        pedroMovement.init(this);
        shooter.init(this, true);
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