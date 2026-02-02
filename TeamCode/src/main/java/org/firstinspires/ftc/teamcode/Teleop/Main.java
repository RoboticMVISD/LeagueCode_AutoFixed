package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.teamcode.AutoAim;
import org.firstinspires.ftc.teamcode.AutoAimJ;
import org.firstinspires.ftc.teamcode.testClasses.PositionalAutoAim;
import org.firstinspires.ftc.teamcode.testClasses.PositionalShooter;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;


@TeleOp (name = "Teleop 2025")
public class Main extends OpMode {

    MovementSystem movementSystem = new MovementSystem();
    Intake intake = new Intake();
    Shooter shooter = new Shooter();
    PositionalShooter posShooter = new PositionalShooter();

    AutoAimJ autoAimJ = new AutoAimJ();
    public static PositionalAutoAim posAutoAim = new PositionalAutoAim();


    @Override
    public void init()
    {
        intake.init(this);
        movementSystem.init(this);
        posShooter.init(this);
        posAutoAim.init(this);
    }

    @Override
    public void loop(){
        intake.loop();
        movementSystem.loop();
        posAutoAim.loop();

        try {
            posShooter.loop();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        telemetry.addData("Is AutoAim on? ", PositionalAutoAim.aimEnabled);
        telemetry.addData("Is AutoDistancing on?", PositionalAutoAim.launcherRequested);
        telemetry.addData("Current POS: ", PositionalShooter.turretRotator.getPosition());
        telemetry.addData("Current VELOCITY: ", PositionalShooter.rightShooter.getVelocity());
    }
    public void stop(){
        autoAimJ.stop();
    }


}