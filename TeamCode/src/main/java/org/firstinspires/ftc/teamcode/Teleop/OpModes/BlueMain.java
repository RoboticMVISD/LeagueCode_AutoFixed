package org.firstinspires.ftc.teamcode.Teleop.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.teamcode.Teleop.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.Teleop.SubSystems.MovementSystems.PedroMovementBlue;
import org.firstinspires.ftc.teamcode.Teleop.SubSystems.Shooter;


@TeleOp (name = "Main-Blue")
public class BlueMain extends OpMode {

    Intake intake = new Intake();
    Shooter shooter = new Shooter();
    PedroMovementBlue pedroMovementBlue = new PedroMovementBlue();

    @Override
    public void init()
    {
        intake.init(this);
        Shooter.init(this, false);
        pedroMovementBlue.init(this);
    }

    @Override
    public void start(){
        pedroMovementBlue.start();
    }

    @Override
    public void loop(){
        intake.loop();
        pedroMovementBlue.loop();

        try {
            shooter.loop();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }


}