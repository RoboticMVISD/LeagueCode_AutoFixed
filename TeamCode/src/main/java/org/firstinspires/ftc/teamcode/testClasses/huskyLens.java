package org.firstinspires.ftc.teamcode.testClasses;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Teleop.Shooter;

public class huskyLens {

    HuskyLens huskyLens;
    OpMode op;

    public void init(OpMode OP){
        op = OP;

        huskyLens = op.hardwareMap.get(HuskyLens.class, "huskyLens");
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
    }

    public void loop(){
        autoAim();
    }

    public void autoAim(){
        HuskyLens.Block[] blocks = huskyLens.blocks();
        op.telemetry.addLine("Number of Tags Recognized: " + blocks.length);

  if (blocks.length <= 2)
            for (HuskyLens.Block block : blocks){
                if (block.x > 161){
                    Shooter.turretRotator.setPower(.2);
                } else if (block.x < 159){
                    Shooter.turretRotator.setPower(-.2);
                } else {
                    Shooter.turretRotator.setPower(0);
                }
         }

        op.telemetry.update();
    }
}
