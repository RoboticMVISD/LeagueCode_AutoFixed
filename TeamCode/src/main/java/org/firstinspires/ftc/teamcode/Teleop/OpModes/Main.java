package org.firstinspires.ftc.teamcode.Teleop.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.teamcode.Teleop.SubSystems.AutoAim;
import org.firstinspires.ftc.teamcode.Teleop.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.Teleop.SubSystems.Movement;
import org.firstinspires.ftc.teamcode.Teleop.SubSystems.PedroMovement;
import org.firstinspires.ftc.teamcode.Teleop.SubSystems.Shooter;


@TeleOp (name = "Teleop 2025")
public class Main extends OpMode {

    Intake intake = new Intake();
    Shooter shooter = new Shooter();
    public static AutoAim autoAim = new AutoAim();
    Movement movement = new Movement();
    PedroMovement pedroMovement = new PedroMovement();

    @Override
    public void init()
    {
        intake.init(this);
        Shooter.init(this, false);
        //autoAim.init(this);
        //movement.init(this.hardwareMap);
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
       // movement.drive(-gamepad1.left_stick_y * .85, gamepad1.left_stick_x * .85, gamepad1.right_stick_x * .85);

        try {
            shooter.loop();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        telemetry.addData("Auto Aim Enabled", autoAim.aimEnabled);
        telemetry.addData("Auto Distancing Enabled", autoAim.launcherRequested);
    }


}