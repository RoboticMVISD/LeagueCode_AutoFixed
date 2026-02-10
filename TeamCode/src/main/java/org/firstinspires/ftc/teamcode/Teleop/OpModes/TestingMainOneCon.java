package org.firstinspires.ftc.teamcode.Teleop.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.teamcode.Teleop.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.Teleop.SubSystems.MovementSystems.Movement;
import org.firstinspires.ftc.teamcode.Teleop.SubSystems.MovementSystems.PedroMovementTestandGeneral;
import org.firstinspires.ftc.teamcode.Teleop.SubSystems.Shooter;


@TeleOp (name = "Main-Testing-OneCon")
public class TestingMainOneCon extends OpMode {

    PedroMovementTestandGeneral pedroMTG = new PedroMovementTestandGeneral();
    Intake intake = new Intake();
    Shooter shooter = new Shooter();
    Movement movement = new Movement();

    @Override
    public void init()
    {
        intake.init(this);
        //pedroMTG.init(this);
        shooter.init(this, true);
        Movement.init(this.hardwareMap);
    }

    @Override
    public void start(){
        //pedroMTG.start();
    }

    @Override
    public void loop(){
        intake.loop();
        //pedroMTG.loop();
        movement.drive(this.gamepad1.left_stick_y, -this.gamepad1.left_stick_x, -this.gamepad1.right_stick_x);

        try {
            shooter.loop();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }


}