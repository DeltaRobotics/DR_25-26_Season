package org.firstinspires.ftc.teamcode.Custom;

//two face teleop
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "Gen0Teleop")
//@Disabled

public class Gen0Teleop extends LinearOpMode {
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
    private double intakePower = 0;

    public final double INTAKE_ON_POWER = 0.75;
    public final double SHOOTER_IDLE_POWER = 0.45;
    public final double SHOOTER_FULL_POWER = 0.80;


    public Servo kicker = null;
    public DcMotor intake = null;
    public DcMotor shooter = null;


    double time = 0;
    public boolean timerInitted = false;
    public boolean timerInit = false;
    public boolean timerInit3 = false;
    public boolean timerInit4 = false;
    public boolean timerInit5 = false;


    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        Gen0Hardwaremap robot = new Gen0Hardwaremap(hardwareMap);
        Gen0MechanismHardwareMap mechanism = new Gen0MechanismHardwareMap(hardwareMap);

        kicker = hardwareMap.servo.get("kicker");

        while (!isStarted() && !isStopRequested()) {

            kicker.setPosition(0);

            //intake.setTargetPosition(0);
            //intake.setPower(.2);
            //intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //shooter.setTargetPosition(0);
            //shooter.setPower(.2);
            //shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }

        while (opModeIsActive()) {

            robot.mecanumDrive(gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, .8);

            if (gamepad2.dpad_down && buttonDD) {

                mechanism.shooterOFF();

                buttonDD = false;
            }

            if (!gamepad2.dpad_down && !buttonDD) {

                buttonDD = true;
            }

            if (gamepad1.right_bumper && buttonRB || timerInitted) {

                if (kicker.getPosition() == 0) {

                    kicker.setPosition(.6);
                    mechanism.intake.setPower(0);
                    time = timer.milliseconds() + 250;
                    timerInitted=true;

                }
                if (time < timer.milliseconds() && timerInitted){
                    kicker.setPosition(0);
                    mechanism.intake.setPower(.9);
                    timerInitted=false;
                }


                buttonRB = false;
            }
            if (!gamepad1.right_bumper && !buttonRB) {

                buttonRB = true;
            }

            if (gamepad1.b && buttonB) {

                buttonB = false;

                mechanism.intake.setPower(mechanism.intake.getPower() * -1);
            }
            else if (!gamepad1.b) {
                buttonB = true;
            }

            if (gamepad1.x && buttonX) {

                buttonX = false;

                if (mechanism.intake.getPower() == 0) {
                    mechanism.intake.setPower(INTAKE_ON_POWER);
                }
                else {
                    mechanism.intake.setPower(0);
                }
            }
            else if (!gamepad1.x) {
                buttonX = true;
            }

            if (gamepad1.b && buttonB) {

                buttonB = false;

                mechanism.intake.setPower(mechanism.intake.getPower() * -1);
            }
            else if (!gamepad1.b) {
                buttonB = true;
            }

            //add shooter shutdown button later

            if (gamepad1.left_bumper && buttonLB) {

                buttonLB = false;

                if (!mechanism.onOff) {
                    mechanism.shooterON();
                }
                else {
                    mechanism.shooterOFF();
                }
            }
            else if (!gamepad1.left_bumper) {
                buttonLB = true;
            }

            telemetry.addData("shooterRPM", mechanism.shooterRPM());
            telemetry.addData("kickerPos", kicker.getPosition());
            telemetry.update();


        }
    }
}