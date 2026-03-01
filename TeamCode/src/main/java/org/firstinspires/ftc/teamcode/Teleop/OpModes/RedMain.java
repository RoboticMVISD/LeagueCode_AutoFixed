package org.firstinspires.ftc.teamcode.Teleop.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.teamcode.Teleop.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.Teleop.SubSystems.MovementSystems.PedroMovementRed;
import org.firstinspires.ftc.teamcode.Teleop.SubSystems.MovementSystems.RedMovement;
import org.firstinspires.ftc.teamcode.Teleop.SubSystems.Shooter;


@TeleOp (name = "Main-Red")
public class RedMain extends OpMode {

    Intake intake = new Intake();
    Shooter shooter = new Shooter();
    RedMovement redMovement = new RedMovement();

    @Override
    public void init()
    {
        intake.init(this);
        Shooter.init(this, false);
        redMovement.init(this);
    }

    @Override
    public void start(){
        redMovement.start();
    }

    @Override
    public void loop(){
        intake.loop();
        redMovement.loop();

        try {
            shooter.loop();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }


}