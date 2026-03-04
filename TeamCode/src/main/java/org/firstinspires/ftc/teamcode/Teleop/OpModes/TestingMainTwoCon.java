package org.firstinspires.ftc.teamcode.Teleop.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.teamcode.Teleop.SubSystems.CameraSystems.AutoAim_Distance;
import org.firstinspires.ftc.teamcode.Teleop.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.Teleop.SubSystems.MovementSystems.RedMove;
import org.firstinspires.ftc.teamcode.Teleop.SubSystems.Shooter;


@TeleOp (name = "Main-Testing-TwoCon")
public class TestingMainTwoCon extends OpMode {

    Intake intake = new Intake();
    Shooter shooter = new Shooter();
    RedMove redMovement = new RedMove();
    AutoAim_Distance autoAim = new AutoAim_Distance();

    @Override
    public void init()
    {
        redMovement.init(this);
        Intake.init(this);
        Shooter.init(this, false);
        autoAim.init(this);
    }

    @Override
    public void start(){
        redMovement.start();
    }

    @Override
    public void loop(){
        intake.loop();
        redMovement.loop();
        autoAim.loop(false, true);

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