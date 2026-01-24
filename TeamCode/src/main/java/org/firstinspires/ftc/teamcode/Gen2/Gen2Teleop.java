package org.firstinspires.ftc.teamcode.Gen2;

//two face teleop
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.concurrent.TimeUnit;


@TeleOp(name = "Gen2Teleop")
//@Disabled

public class Gen2Teleop extends LinearOpMode {
    public boolean buttonY = true;
    public boolean buttonX = true;
    public boolean buttonA = true;
    public boolean buttonB = true;
    public boolean buttonLB = true;
    public boolean buttonLT = true;
    public boolean buttonRB = true;
    public boolean buttonRT = true;
    public boolean buttonDR = true;
    public boolean buttonDL = true;
    public boolean buttonDU = true;
    public boolean buttonDD = true;

    public boolean button2Y = true;
    public boolean button2X = true;
    public boolean button2A = true;
    public boolean button2B = true;
    public boolean button2LB = true;
    public boolean button2LT = true;
    public boolean button2RB = true;
    public boolean button2RT = true;
    public boolean button2DR = true;
    public boolean button2DL = true;
    public boolean button2DU = true;
    public boolean button2DD = true;

    public AprilTagDetection Detection = null;

    double time = 0;
    public boolean timerInitted = false;

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        Gen2Hardwaremap robot = new Gen2Hardwaremap(hardwareMap);

        robot.L_swingythingy.setPosition(robot.L_swingy_Thingy_Close);
        robot.R_swingythingy.setPosition(robot.R_swingy_Thingy_Close);

        robot.transfer.setPower(0);

        robot.R_feeder.setPower(0);
        robot.L_feeder.setPower(0);

        robot.initAprilTag(hardwareMap);

        robot.hoodDown();


        waitForStart();


        while (opModeIsActive()) {

            robot.mecanumDrive(gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, 1);
            robot.turret();

            if(gamepad1.left_bumper && buttonLB){

                robot.intake();

                buttonLB = false;
            }

            if(!gamepad1.left_bumper && !buttonLB){

                buttonLB = true;
            }

            if(gamepad1.left_trigger > 0.5 && buttonLT ){

                robot.outTake();

                telemetry.addData("transfer", robot.transfer.getPower());

                buttonLT = false;
            }

            if(gamepad1.left_trigger < 0.5 && !buttonLT){

                buttonLT = true;

            }

            if(gamepad1.b && buttonB){

                robot.stopIntake();

                buttonB = false;
            }

            if(!gamepad1.b && !buttonB){

                buttonB = true;

            }

            if(gamepad1.right_bumper){

                robot.shoot();

                telemetry.addData("transfer", robot.transfer.getPower());

                buttonRB = false;

            }

            if(!gamepad1.right_bumper && !buttonRB){

                buttonRB = true;

            }

            if(robot.timerInitted[0]){

                robot.shoot();
            }





            //                      Driver 2


            //Bringing the hood up
            if(gamepad2.dpad_up && button2DU){
                robot.PIDShooter.setP(robot.PIDShooter.getP() + 0.005);

                button2DU = false;
            }

            if(!gamepad2.dpad_up && !button2DU) {

                button2DU = true;
            }

            //Decreasing the RPM
            if(gamepad2.dpad_down && button2DD){

                robot.PIDShooter.setP(robot.PIDShooter.getP() - 0.005);

                button2DD = false;
            }

            if(!gamepad2.dpad_down && !button2DD){

                button2DD = true;

            }

            //Bringing the hood down
            if(gamepad2.dpad_right && button2DR){

                robot.PIDShooter.setD(robot.PIDShooter.getD() - 0.000005);

                button2DR = false;
            }

            if(!gamepad2.dpad_right && !button2DR){

                button2DR = true;
            }


            //increasing RPM
            if(gamepad2.dpad_left && button2DL){

                robot.PIDShooter.setD(robot.PIDShooter.getD() + 0.000005);

                button2DL = false;
            }

            if(!gamepad2.dpad_left && !button2DL){

                button2DL = true;
            }



            //calling to track the blue AprilTag
            if(gamepad2.x && button2X){
                robot.blue = true;

                button2X = false;
            }

            if(!gamepad2.x && !button2X){

                button2X = true;
            }

            //calling to track the red AprilTag
            if(gamepad2.b && button2B){
                robot.blue = false;

                button2B = false;
            }

            if(!gamepad2.b && !button2B){

                button2B = true;
            }






            // Race condition where getDetections() is not empty and then when it is read,
            // becomes empty because its data updated.



            Detection = robot.getDetection();
            if(Detection != null){
                telemetry.addData("bearing", Detection.ftcPose.bearing);
                telemetry.addData("Range", Detection.ftcPose.range);
            }
            else{
                telemetry.addData("bearing", "Tag Not Detected");
                telemetry.addData("Range", "Tag Not Detected");
            }

            telemetry.addData("min exposure ", robot.myExposureControl.getMinExposure(TimeUnit.MILLISECONDS) );
            telemetry.addData("max exposure ", robot.myExposureControl.getMaxExposure(TimeUnit.MILLISECONDS) );
            telemetry.addData("current exposure ", robot.myExposureControl.getExposure(TimeUnit.MILLISECONDS) );

            telemetry.addData("current shooter P ", String.valueOf(robot.PIDShooter.getP()));
            telemetry.addData("current shooter D ", String.valueOf(robot.PIDShooter.getD()));


            telemetry.addData("current turret P ", String.valueOf(robot.PIDTurret.getP()));
            telemetry.addData("current turret D ", String.valueOf(robot.PIDTurret.getD()));

            telemetry.addData("target RPM ", robot.targetRPM);
            telemetry.addData("shooterRPM", robot.shooterRPM());
            telemetry.addData("hood position ", robot.hood_pos);
            telemetry.addData("shooter Position right", robot.R_shooter.getCurrentPosition());
            telemetry.addData("real shooter power", robot.R_shooter.getPower());
            telemetry.addData("error", robot.error);
            telemetry.addData("power", robot.setting_ShooterRPM());
            telemetry.addData("hood", robot.hood.getPosition());
            telemetry.addData("heading", robot.hood.getPosition());
            telemetry.addData("leftOdo", robot.motorLF.getCurrentPosition());
            telemetry.addData("rightOdo", robot.motorRB.getCurrentPosition());
            telemetry.addData("strafeOdo", robot.motorLB.getCurrentPosition());
            telemetry.update();

        }
        robot.intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}