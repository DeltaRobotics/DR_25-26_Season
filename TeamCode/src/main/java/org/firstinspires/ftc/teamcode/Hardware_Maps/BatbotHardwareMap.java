package org.firstinspires.ftc.teamcode.Hardware_Maps;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class BatbotHardwareMap {

    //FtcDashboard dashboard = FtcDashboard.getInstance();
    //drive motors
    public DcMotor motorRF = null;
    public DcMotor motorLF = null;
    public DcMotor motorRB = null;
    public DcMotor motorLB = null;

    //odometry encoder objects
    public DcMotor leftEncoder = null;
    public DcMotor rightEncoder = null;
    public DcMotor perpendicularEncoder = null;

    public DcMotor[] odometers = new DcMotor[3];
    public DcMotor[] drive = new DcMotor[4];
    VoltageSensor ControlHub_VoltageSensor = null;

    public double moveSpeed = 0.5;
    public double turnSpeed = 0.5;

    public double moveAccuracy  = 1;
    public double angleAccuracy = Math.toRadians(1);

    boolean button1 = false;
    boolean button2 = false;

    public static boolean timerInitted = false;

    private final int READ_PERIOD = 1;

    private HuskyLens huskyLens;
    public final double CLAW_OPEN = 1;
    public final double CLAW_CLOSE = .6;

    public final double WRIST_UP = .5;
    public final double WRIST_DOWN = .65;

    public final double SLIDES_OUT = .75;
    public final double SLIDES_INSIDE = .905;

    public static ElapsedTime currentTime = new ElapsedTime();

    public BatbotHardwareMap(HardwareMap ahwMap)
    {


        //drive motors
        motorRF = ahwMap.dcMotor.get("motorRF");
        motorLF = ahwMap.dcMotor.get("motorLF");
        motorRB = ahwMap.dcMotor.get("motorRB");
        motorLB = ahwMap.dcMotor.get("motorLB");

        //drive motors and odometry encoders
        motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //motorRB.setDirection(DcMotorSimple.Direction.REVERSE);
        //motorLB.setDirection(DcMotorSimple.Direction.REVERSE);

        motorRF.setPower(0);
        motorLF.setPower(0);
        motorRB.setPower(0);
        motorLB.setPower(0);


        //odometry init (use the motors objects that the odometers are plugged into)
        leftEncoder = motorLB;
        rightEncoder = motorLF;
        perpendicularEncoder = motorRF;

        odometers[0] = leftEncoder;
        odometers[1] = rightEncoder;
        odometers[2] = perpendicularEncoder;

        drive[0] = motorRF;
        drive[1] = motorRB;
        drive[2] = motorLB;
        drive[3] = motorLF;

        ControlHub_VoltageSensor = ahwMap.get(VoltageSensor.class, "Control Hub");
    }

    public void mecanumDrive(double forward, double strafe, double heading, double speed){

        motorRF.setPower((((forward - strafe) * 1) - (heading * 1)) * speed);
        motorRB.setPower((((forward + strafe) * 1) - (heading * 1)) * speed);
        motorLB.setPower((((forward - strafe) * 1) + (heading * 1)) * speed);
        motorLF.setPower((((forward + strafe) * 1) + (heading * 1)) * speed);
    }

    public void resetDriveEncoders()
    {
        motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public double angleWrapRad(double angle)
    {
        while (angle > Math.PI)
        {
            angle -= Math.PI * 2;
        }
        while (angle < -Math.PI)
        {
            angle += Math.PI * 2;
        }

        return angle;
    }


    //modifier for the speed attributes of the robot when moving in auto
    public void changeSpeed(double mSpeed, double tSpeed){
        moveSpeed = mSpeed;
        turnSpeed = tSpeed;
    }

    //modifier for accuracy variables for the robot in auto
    public void changeAccuracy(double mAccuracy, double aAccuracy){
        moveAccuracy = mAccuracy;
        angleAccuracy = aAccuracy;
    }

    public void duelServoController(double target, Servo servoLeft, Servo servoRight){
        servoLeft.setPosition(Math.abs(1-target));
        servoRight.setPosition(target);
    }

    public void servoFineAdjust(Servo s, boolean increase, boolean decrease, double increment){
        if (increase && button1){
            s.setPosition(s.getPosition() + increment);
            button1 = false;
        }
        else if (decrease && button2){
            s.setPosition(s.getPosition() - increment);
            button2 = false;
        }
        else if (!increase && !button1){
            button1 = true;
        }
        else if (!decrease && !button2){
            button2 = true;
        }
    }

    public void runOpMode(){}

    public boolean boolTimer (double time){
        return currentTime.milliseconds() > time;
    }

    public double timerInit(int t){

        double ti = currentTime.milliseconds() + t;
        //timerInitted = true;

        return ti;

    }
}



