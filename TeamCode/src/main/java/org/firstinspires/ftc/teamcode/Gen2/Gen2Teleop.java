package org.firstinspires.ftc.teamcode.Gen2;

//two face teleop
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;


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

            robot.R_shooter.setPower(robot.setting_ShooterRPM());
            robot.L_shooter.setPower(robot.setting_ShooterRPM());

            //Close range shooting
            if(gamepad1.dpad_right && buttonDR){

                robot.hoodDown();

                telemetry.addData("transfer", robot.transfer.getPower());

                buttonDR = false;
            }

            if(!gamepad1.dpad_right && !buttonDR){

                buttonDR = true;
            }

            //Long range shooting
            if(gamepad1.dpad_left && buttonDL){

                robot.hoodUp();

                telemetry.addData("transfer", robot.transfer.getPower());

                buttonDL = false;
            }

            if(!gamepad1.dpad_left && !buttonDL){

                buttonDL = true;
            }

            if(gamepad1.dpad_down && buttonDD){

                robot.hoodMid();

                telemetry.addData("transfer", robot.transfer.getPower());

                buttonDD = false;
            }

            if(!gamepad1.dpad_down && !buttonDD){

                buttonDD = true;
            }

            if(robot.aprilTag.getDetections().size() > 0){

                Detection = robot.aprilTag.getDetections().get(0);

                if(Detection != null){
                    telemetry.addData("bearing", Detection.ftcPose.bearing);
                }
            }

            telemetry.addData("shooter Position right", robot.R_shooter.getCurrentPosition());
            telemetry.addData("shooter Position left", robot.L_shooter.getCurrentPosition());
            telemetry.addData("real shooter power", robot.R_shooter.getPower());
            telemetry.addData("error", robot.error);
            telemetry.addData("power", robot.setting_ShooterRPM());
            telemetry.addData("shooterRPM", robot.shooterRPM());
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