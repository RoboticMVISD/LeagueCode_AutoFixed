package org.firstinspires.ftc.teamcode.Teleop.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.teamcode.Teleop.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.Teleop.SubSystems.Movement;
import org.firstinspires.ftc.teamcode.Teleop.SubSystems.Shooter;


@TeleOp (name = "Teleop Testing")
public class TestingMain extends OpMode {

    Movement movement = new Movement();
    Intake intake = new Intake();
    Shooter shooter = new Shooter();

    @Override
    public void init()
    {
        intake.init(this);
        movement.init(this.hardwareMap);
        shooter.init(this, true);
    }

    @Override
    public void loop(){
        intake.loop();
        movement.drive(-gamepad1.left_stick_y * .85, gamepad1.left_stick_x * .85, gamepad1.right_stick_x * .85);

        try {
            shooter.loop();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }


}