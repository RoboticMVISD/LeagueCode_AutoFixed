package org.firstinspires.ftc.teamcode.Teleop.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.teamcode.Teleop.SubSystems.AutoAim;
import org.firstinspires.ftc.teamcode.Teleop.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.Teleop.SubSystems.Movement;
import org.firstinspires.ftc.teamcode.Teleop.SubSystems.Shooter;


@TeleOp (name = "Teleop 2025")
public class Main extends OpMode {

    Intake intake = new Intake();
    Shooter shooter = new Shooter();
    public static AutoAim autoAim = new AutoAim();
    Movement moveTest = new Movement();

    @Override
    public void init()
    {
        intake.init(this);
        //movementSystem.init(this);
        Shooter.init(this, false);
        autoAim.init(this);
        moveTest.init(this.hardwareMap);
    }

    @Override
    public void loop(){
        intake.loop();
        //movementSystem.loop();
        autoAim.loop();
        moveTest.drive(-gamepad1.left_stick_y * .85, gamepad1.left_stick_x * .85, gamepad1.right_stick_x * .85);

        try {
            shooter.loop();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        telemetry.addData("Auto Aim Enabled", autoAim.aimEnabled);
        telemetry.addData("Auto Distancing Enabled", autoAim.launcherRequested);
    }


}