package org.firstinspires.ftc.teamcode.Custom;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="emptyOpmode")
@Disabled

public class emptyOpMode extends LinearOpMode
{

    @Override
    public void runOpMode() throws InterruptedException
    {

        waitForStart();

        while (opModeIsActive())
        {

        }
    }
}
