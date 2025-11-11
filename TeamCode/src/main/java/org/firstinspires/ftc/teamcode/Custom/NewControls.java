package org.firstinspires.ftc.teamcode.Custom;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


@TeleOp(name="NewControls")
@Disabled

public class NewControls extends LinearOpMode
{

    private double currentX = 0;

    private double previousX = 0;

    private double currentY = 0;

    private double previousY = 0;

    private double stickRotation = 0;

    private YawPitchRollAngles imuHeading = null;


    @Override
    public void runOpMode() throws InterruptedException {
        SoftwareTestBot robot = new SoftwareTestBot(hardwareMap);
        robot.resetHeading();

        waitForStart();

        while (opModeIsActive()){

            robot.mecanumDrive(-gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x, .75);


            telemetry.addData("Heading", robot.getHeading());
            telemetry.update();

        }
    }
}
