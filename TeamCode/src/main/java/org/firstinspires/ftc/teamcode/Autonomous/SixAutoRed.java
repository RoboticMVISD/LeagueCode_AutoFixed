package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Teleop.Intake;
import org.firstinspires.ftc.teamcode.Teleop.MovementSystem;
import org.firstinspires.ftc.teamcode.Teleop.Shooter;

@Autonomous (name = "6AutoRed")
public class SixAutoRed extends OpMode{
    ElapsedTime timer;
    OpMode op;
    Boolean timerDone;


    enum autoState{
        SPINNING,
        READY
    }

    autoState as;

    public void init(){
        timerDone = true;

        op = this;

        MovementSystem.init(this);
        Intake.init(this);
        Shooter.init(this);

        as = autoState.SPINNING;
    }

    public void start(){
        timer = new ElapsedTime();
        timer.reset();
        timer.startTime();
    }

    public void loop(){
        while (timerDone){
            startAuto();
        }
        op.terminateOpModeNow();
    }

    public void startAuto(){
        if (timer.seconds() < .8){
            MovementSystem.setMotorPowerAll(-.4);
        } else if (timer.seconds() >= .8 && timer.seconds() < 1) {
            MovementSystem.setMotorPowerAll(0);
        } else if(timer.seconds() >= 1  && timer.seconds() <= 4) {

            Shooter.setShooterPower(Shooter.SPIN_UP_VELOCITY_SHORTRANGE + 20);

            if (Shooter.rightShooter.getVelocity() >= Shooter.SPIN_UP_VELOCITY_SHORTRANGE - 10 && Shooter.rightShooter.getVelocity() <= Shooter.SPIN_UP_VELOCITY_SHORTRANGE + 10) {
                as = autoState.READY;
            }

            if (as == autoState.READY && timer.seconds() < 4){
                Intake.setBothIntakePower(1);
            } else if (as == autoState.READY && timer.seconds() == 4){
                Shooter.setShooterPower(0);
                Intake.setBothIntakePower(0);
                as = autoState.SPINNING;
            }
        } else if (timer.seconds() > 4  && timer.seconds() < 4.4){
            MovementSystem.setMotorPowerAll(.4,.4,-.4,-.4);
            Intake.setBothIntakePower(0);
        } else if (timer.seconds() >= 4.4  && timer.seconds() < 4.5){
            MovementSystem.setMotorPowerAll(0);
            Shooter.rightShooter.setVelocity(0);
            Shooter.leftShooter.setVelocity(0);
        }else if (timer.seconds() >= 4.5  && timer.seconds() <= 5.1){
            MovementSystem.setMotorPowerAll(-.4,.4,.4,-.4);
        }else if (timer.seconds() > 5.1  && timer.seconds() <= 6.8){
            Shooter.setShooterPower(Shooter.BALL_REVERSE_SPEED * 1.5);
            MovementSystem.setMotorPowerAll(.4);
           Intake.setBothIntakePower(1);
           if (timer.seconds() > 6.2) {
               Intake.secondaryIntake.setPower(-.8);
           }
        }else if (timer.seconds() > 6.8  && timer.seconds() <= 6.9){
            Shooter.setShooterPower(0);
            Intake.setBothIntakePower(0);
        }else if (timer.seconds() > 6.9  && timer.seconds() <= 7.4){
            MovementSystem.setMotorPowerAll(-.4);
            }else if (timer.seconds() > 7.4  && timer.seconds() <= 9){
            MovementSystem.setMotorPowerAll(.4,-.4,-.4,.4);
        }else if (timer.seconds() > 9  && timer.seconds() <= 9.2){
            MovementSystem.setMotorPowerAll(0);
        }else if (timer.seconds() > 9.2  && timer.seconds() <= 9.6){
            MovementSystem.setMotorPowerAll(-.4,-.4,.4,.4);
        }else if (timer.seconds() > 9.6  && timer.seconds() <= 10.5){
            MovementSystem.setMotorPowerAll(.3);
        }else if (timer.seconds() > 10.5  && timer.seconds() <= 11.3){
            MovementSystem.setMotorPowerAll(-.4);
        } else if (timer.seconds() > 11.3  && timer.seconds() <= 11.4){
            MovementSystem.setMotorPowerAll(0);
        } else if(timer.seconds() >= 11.4  && timer.seconds() <= 13.8) {

            Shooter.setShooterPower(Shooter.SPIN_UP_VELOCITY_SHORTRANGE);

            if (Shooter.rightShooter.getVelocity() >= Shooter.SPIN_UP_VELOCITY_SHORTRANGE - 10 && Shooter.rightShooter.getVelocity() <= Shooter.SPIN_UP_VELOCITY_SHORTRANGE + 10) {
                as = autoState.READY;
            }

            if (as == autoState.READY && timer.seconds() < 13.8 && timer.seconds() > 11.8){
                Intake.setBothIntakePower(1);
            } else if (as == autoState.READY && timer.seconds() == 13.8){
                Shooter.setShooterPower(0);
                Intake.setBothIntakePower(0);
                as = autoState.SPINNING;
            }
        }else if (timer.seconds() > 13.8  && timer.seconds() <= 14.2){
            Intake.intake.setPower(0);
            Intake.secondaryIntake.setPower(0);
            Shooter.rightShooter.setVelocity(0);
            Shooter.leftShooter.setVelocity(0);
            MovementSystem.setMotorPowerAll(.35,.35,-.35,-.35);
        } else if (timer.seconds() > 14.2 && timer.seconds() <= 14.3){
            MovementSystem.setMotorPowerAll(0);
        } else if (timer.seconds() > 14.3 && timer.seconds() <= 17){
            MovementSystem.setMotorPowerAll(-.35,.35,.35,-.35);
        }else {
            MovementSystem.setMotorPowerAll(0);
            Shooter.setShooterPower(0);
            Intake.setBothIntakePower(0);
            timerDone = false;
        }
    }
    }


