package org.firstinspires.ftc.teamcode.Teleop.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.teamcode.Teleop.SubSystems.CameraSystems.AutoAim_Distance;
import org.firstinspires.ftc.teamcode.Teleop.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.Teleop.SubSystems.MovementSystems.RedMovement;
import org.firstinspires.ftc.teamcode.Teleop.SubSystems.Shooter;


@TeleOp (name = "Main-Testing-OneCon")
public class TestingMainOneCon extends OpMode {

    Intake intake = new Intake();
    Shooter shooter = new Shooter();
    RedMovement pedroMovementRed = new RedMovement();
    public static AutoAim_Distance autoAim = new AutoAim_Distance();

    @Override
    public void init()
    {
        Intake.init(this);
        Shooter.init(this, true);
        pedroMovementRed.init(this);
        autoAim.init(this);
    }

    @Override
    public void start(){
        pedroMovementRed.start();
    }

    @Override
    public void loop(){
        intake.loop();
        pedroMovementRed.loop();
        autoAim.loop(true);

        try {
            AutoAim_Distance.update();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        try {
            shooter.loop();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }


}