package org.firstinspires.ftc.teamcode.Teleop.SubSystems.MovementSystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;

public class FieldCentricDrive {

    private final Follower follower;

    // Optional speed scaling
    private double driveSpeedMultiplier = .8;
    private double turnDamper = 0.6;

    public FieldCentricDrive(Follower follower) {
        this.follower = follower;
    }


    public void drive(Gamepad gamepad) {

        // Get joystick input
        double y = -gamepad.left_stick_y;   // Forward is negative on gamepad
        double x = gamepad.left_stick_x;
        double turn = -gamepad.right_stick_x;

        // Get robot heading from Pedro
        Pose pose = follower.getPose();
        double heading = pose.getHeading();

        // Rotate joystick vector into field frame
        double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

        // Apply optional speed scaling
        rotX *= driveSpeedMultiplier;
        rotY *= driveSpeedMultiplier;
        turn *= turnDamper;

        // Send to robot-centric drive system
        Movement.drive(rotY, rotX, turn);
    }

}
