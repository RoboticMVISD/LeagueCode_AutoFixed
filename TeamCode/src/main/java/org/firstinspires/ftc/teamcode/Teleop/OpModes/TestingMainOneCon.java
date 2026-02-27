package org.firstinspires.ftc.teamcode.Teleop.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.teamcode.Teleop.SubSystems.CameraSystems.BetterLimelightAutoAim;
import org.firstinspires.ftc.teamcode.Teleop.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.Teleop.SubSystems.MovementSystems.Movement;
import org.firstinspires.ftc.teamcode.Teleop.SubSystems.MovementSystems.PedroMovementTestandGeneral;
import org.firstinspires.ftc.teamcode.Teleop.SubSystems.Shooter;
import org.firstinspires.ftc.teamcode.Teleop.SubSystems.autoPathingSystems.AutoPathingTeleRed;


@TeleOp (name = "Main-Testing-OneCon")
public class TestingMainOneCon extends OpMode {

    Intake intake = new Intake();
    Shooter shooter = new Shooter();
    AutoPathingTeleRed autoPathRed = new AutoPathingTeleRed();
    public static BetterLimelightAutoAim autoAim = new BetterLimelightAutoAim();

    @Override
    public void init()
    {
        Intake.init(this);
        Shooter.init(this, true);
        autoPathRed.init(this);
        autoAim.init(this);
    }

    @Override
    public void start(){
        autoPathRed.start();
    }

    @Override
    public void loop(){
        intake.loop();
        autoPathRed.loop();
        autoAim.loop();

        try {
            BetterLimelightAutoAim.update();
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