package org.firstinspires.ftc.teamcode.Teleop.SubSystems.CameraSystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Teleop.SubSystems.MovementSystems.Movement;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp (name = "Auto Align Test")
public class AutoAlign extends OpMode {

    private final WebCam webCam = new WebCam();
    private Movement movement = new Movement();
    // ------------- PD Controller ----------------
    double kP = 0.002;
    double error = 0;
    double lastError = 0;
    double goalX; //Offset
    double angleTolerance = 0.3; //Decreases if farther away
    double kD = 0.00001;
    double curTime = 0;
    double lastTime = 0;

    //------------ Driving SetUp ----------------
    double forward, strafe, rotate;

    //--------------- Controller Based PD Tuning ------------
    double [] stepSizes = {1.0, .1, 0.001, 0.0001};
    int stepIndex = 2;
    Telemetry tele = this.telemetry;

    public void init(){
        webCam.init(this);
        movement.init(this.hardwareMap);
    }

    public void start(){
        resetRuntime();
        curTime = getRuntime();
    }

    public void loop(){
        //------------- Get Drive Inputs ----------------
        forward = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        //------ get AprilTag Info --------
        webCam.update();
        AprilTagDetection id20 = webCam.getTagBySpecificId(20);

        // ---------- Auto Align Rotation Logic -----------
        if (gamepad1.left_trigger > 0.3){
            //Ensures we are seeing a tag & Pressing Trigger
            if (id20 != null){
                error = goalX - id20.ftcPose.bearing; //tx

                if (Math.abs(error) < angleTolerance){
                    rotate = 0;
                } else {
                    double pTerm = error * kP;

                    curTime = getRuntime();
                    double dT = curTime - lastTime;
                    double dTerm = ((error - lastError) / dT) * kD;

                    rotate = Range.clip(pTerm + dTerm, -4, .4);

                    lastError = error;
                    lastTime = curTime;
                };
            }
            else {
                lastTime = getRuntime();
                lastError = 0;
            }
        } else {
            lastError = 0;
            lastError = getRuntime();
        }

        //Drive Our Motors
        movement.drive(forward, strafe, rotate);

        if (gamepad1.bWasPressed()){
            stepIndex = (stepIndex - 1) % stepSizes.length;
        }

        //Dpad L/R to adjust P gain
        if (gamepad1.dpadLeftWasPressed()){
            kP -= stepSizes[stepIndex];
        }
        if (gamepad1.dpadRightWasPressed()){
            kP += stepSizes[stepIndex];
        }

        //Dpad U/D to change D
        if (gamepad1.dpadUpWasPressed()){
            kD += stepSizes[stepIndex];
        }
        if (gamepad1.dpadDownWasPressed()){
            kD -= stepSizes[stepIndex];
        }

        //------- Telemetry --------
        if (id20 != null){
            if (gamepad1.left_trigger > 0.3){
                tele.addLine("AUTO ALIGN");
            }
            tele.addData("Error", error);
        } else {
            tele.addLine("MANUAL ROTATE MODE");
        }
        tele.addLine("--------------------");
        tele.addData("Tuning P: ", "%.4f (Dpad L/R)", kP);
        tele.addData("Tuning D: ", "%.4f (Dpad U/D)", kD);
        tele.addData("Step Size: ", "%.4f (B button)", stepSizes[stepIndex]);
        webCam.telemetryUpdate();

    }

    public void stop(){
        webCam.stop();
    }

}