package org.firstinspires.ftc.teamcode.Gen2;

//two face teleop
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
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

    public boolean lifting = false;

    public AprilTagDetection Detection = null;



    @Override
    public void runOpMode() throws InterruptedException {
        Gen2Hardwaremap robot = new Gen2Hardwaremap(hardwareMap);

        robot.L_swingythingy.setPosition(robot.L_swingy_Thingy_Close);
        robot.R_swingythingy.setPosition(robot.R_swingy_Thingy_Close);

        robot.transfer.setPower(0);

        robot.L_PTO.setPosition(robot.L_PTO_UP);
        robot.R_PTO.setPosition(robot.R_PTO_UP);

        robot.R_feeder.setPower(0);
        robot.L_feeder.setPower(0);

        robot.R_turret.setPower(0);
        robot.L_turret.setPower(0);

        robot.initAprilTag(hardwareMap);

        robot.hoodDown();

        robot.targetRPM = 300;

        robot.intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);// CHANGE THIS LATER
        robot.intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            if(!lifting) {
                robot.mecanumDrive(gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, 1);
            }

            robot.turret(telemetry);
            robot.R_shooter.setPower(robot.setting_ShooterRPM());
            robot.L_shooter.setPower(robot.setting_ShooterRPM());


            if(gamepad1.left_bumper && buttonLB){

                robot.intake();
                robot.display_state_intaking();

                buttonLB = false;
            }

            if(!gamepad1.left_bumper && !buttonLB){

                buttonLB = true;
            }

            if(gamepad1.left_trigger > 0.5 && buttonLT ){

                robot.outTake();
                robot.display_state_outputting();

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

            if(gamepad1.right_bumper && buttonRB){

                while(gamepad1.right_bumper && buttonRB){

                    robot.teleOpShoot();
                    robot.display_state_shooting();
                }

                robot.transfer.setPower(1);

                robot.L_feeder.setPower(1);
                robot.R_feeder.setPower(-1);

                buttonRB = false;
            }

            if(!gamepad1.right_bumper && !buttonRB){
                buttonRB = true;

            }


            if(gamepad1.dpad_up && buttonDU){

                robot.PIDTurret.setP(robot.PIDTurret.getP() + 0.005);

                //robot.targetRPM = robot.targetRPM + 100;

                buttonDU = false;

            }

            if(!gamepad1.dpad_up && !buttonDU){
                buttonDU = true;
            }

            if(gamepad1.dpad_down && buttonDD){

                robot.PIDTurret.setP(robot.PIDTurret.getP() - 0.005);

                //robot.targetRPM = robot.targetRPM - 100;

                buttonDD = false;
            }

            if(!gamepad1.dpad_down && !buttonDD){
                buttonDD = true;
            }






            if(gamepad1.y && buttonY){

                robot.PIDTurret.setI(robot.PIDTurret.getI() + 0.0001);

                //robot.targetRPM = robot.targetRPM - 100;

                buttonY = false;
            }

            if(!gamepad1.y && !buttonY){
                buttonY = true;
            }

            if(gamepad1.a && buttonA){

                robot.PIDTurret.setI(robot.PIDTurret.getI() - 0.0001);

                //robot.targetRPM = robot.targetRPM - 100;

                buttonA = false;
            }

            if(!gamepad1.a && !buttonA){
                buttonA = true;
            }




            //hood moving up
            if(gamepad1.dpad_left && buttonDL){

                //robot.PIDShooter.setD(robot.PIDShooter.getD() + 0.005);

                robot.PIDTurret.setD(robot.PIDTurret.getD() - 0.0001);

                buttonDL = false;
            }

            if(!gamepad1.dpad_left && !buttonDL){
                buttonDL = true;
            }


            //hood moving down
            if(gamepad1.dpad_right && buttonDR){

                //robot.PIDShooter.setP(robot.PIDShooter.getP() - 0.005);

                robot.PIDTurret.setD(robot.PIDTurret.getD() + 0.0001);

                buttonDR = false;

            }

            if(!gamepad1.dpad_right && !buttonDR){
                buttonDR = true;
            }


            /**
             **************************************************
             *                  DRIVER 2
             **************************************************
             */

            if(lifting){
                robot.mecanumDrive(-gamepad2.right_stick_y, 0, 0, 1);
                robot.display_state_Lifting();
            }

            //shooting for driver 2
            if(gamepad2.right_trigger > 0.5 && button2RT){

                robot.teleOpShoot();
                button2RT = false;
            }

            if(gamepad2.right_trigger < 0.5 && !button2RT){

                button2RT = true;
            }

            //Lifting
            if(gamepad2.start && gamepad2.dpad_up && button2DU){

                lifting = true;

                if(!robot.timerInitted[15]) {//very very first thing to happen
                    robot.timeArray[15] = robot.currentTime.milliseconds();
                    robot.timerInitted[15] = true;
                }

                if (robot.currentTime.milliseconds() > robot.timeArray[15] + 350) {//Last thing to happen
                    robot.display_state_liftActivated();
                    button2DU = false;
                }

                else if (robot.currentTime.milliseconds() > robot.timeArray[15] + 250) {

                    robot.motorRF.setPower(-0.25);
                    robot.motorLF.setPower(-0.25);
                    robot.motorRB.setPower(-0.25);
                    robot.motorLB.setPower(-0.25);
                }

                else {//Second thing to happen

                    robot.L_PTO.setPosition(robot.L_PTO_DOWN);
                    robot.R_PTO.setPosition(robot.R_PTO_DOWN);
                }
            }
            if(!gamepad2.start && !gamepad2.dpad_up && !button2DU){

                button2DU = true;
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

            LLResult result = robot.limelight.getLatestResult();
            Pose3D botpose = result.getBotpose();

            /**

            telemetry.addData("tx", result.getTx());
            telemetry.addData("txnc", result.getTxNC());
            telemetry.addData("ty", result.getTy());
            telemetry.addData("tync", result.getTyNC());

            telemetry.addData("Botpose", botpose.toString());

            // Access classifier results
            List<LLResultTypes.ClassifierResult> classifierResults = result.getClassifierResults();
            for (LLResultTypes.ClassifierResult cr : classifierResults) {
                telemetry.addData("Classifier", "Class: %s, Confidence: %.2f", cr.getClassName(), cr.getConfidence());
            }

            // Access fiducial results
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
            }

            // Access color results
            List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
            for (LLResultTypes.ColorResult cr : colorResults) {
                telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
            }
             */

            telemetry.addData("L_PTO Pos ", robot.L_PTO.getPosition());
            telemetry.addData("R_PTO Pos ", robot.R_PTO.getPosition());

            telemetry.addData("error ", robot.PIDShooter.getPreviousError());

            telemetry.addData("shooter Position right ", robot.R_shooter.getCurrentPosition());

            telemetry.addData("shooterRPM ", robot.shooterRPM());

            telemetry.addData("power ", robot.setting_ShooterRPM());

            telemetry.addData("real shooter power ", robot.R_shooter.getPower());

            telemetry.addData("targetRPM ", robot.targetRPM);

            telemetry.addData("hood ", robot.hood.getPosition());

            telemetry.addData("hood pos ", robot.hood_pos);

            telemetry.addData("Right Turret Power", robot.R_turret.getPower());

            telemetry.addData("tx", robot.limelight.getLatestResult().getFiducialResults().isEmpty() ? "No Target" : robot.limelight.getLatestResult().getFiducialResults().get(0).getTargetXDegrees());

            telemetry.addData("turret Encoder", robot.turretEncoderCounts);

            telemetry.addData("turret P", String.valueOf(robot.PIDTurret.getP()));

            telemetry.addData("turret I", String.valueOf(robot.PIDTurret.getI()));

            telemetry.addData("turret D", String.valueOf(robot.PIDTurret.getD()));

            telemetry.update();

        }
        robot.intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.limelight.stop();
    }
}