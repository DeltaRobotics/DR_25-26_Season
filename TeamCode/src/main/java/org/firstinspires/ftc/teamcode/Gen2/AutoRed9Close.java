package org.firstinspires.ftc.teamcode.Gen2;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;



@Disabled
@Autonomous(name = "AutoRed9Close")
public class AutoRed9Close extends OpMode {

    Gen2Hardwaremap robot = null;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private final Pose startPose = new Pose(113, 114, Math.toRadians(45));
    private final Pose Shooting = new Pose(90, 92, Math.toRadians(45));
    private final Pose firstLineup = new Pose(84, 77, Math.toRadians(0));
    private final Pose firstPickup = new Pose(110, 77, Math.toRadians(0));
    private final Pose secondLineup = new Pose(84, 53, Math.toRadians(0));
    private final Pose secondPickup = new Pose(116, 52, Math.toRadians(0));
    private final Pose secondPickupBack = new Pose(100, 52, Math.toRadians(0));
    private final Pose movingOffLine = new Pose(84, 108, Math.toRadians(30));

    private Path scorePreload, firstLineupPath, firstPickupPath, shootFirstPickupPath,secondLineupPath, secondPickupPath, secondPickupBackPath, shootSecondLineupPath, movingBackPath ;

    public void buildPaths() {

        scorePreload = new Path(new BezierLine(startPose, Shooting));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), Shooting.getHeading());

        firstLineupPath = new Path (new BezierLine(Shooting,firstLineup));
        firstLineupPath.setConstantHeadingInterpolation(firstLineup.getHeading());

        firstPickupPath = new Path (new BezierLine(firstLineup,firstPickup));
        firstPickupPath.setConstantHeadingInterpolation(firstPickup.getHeading());

        shootFirstPickupPath = new Path (new BezierLine(firstPickup,Shooting));
        shootFirstPickupPath.setConstantHeadingInterpolation(Shooting.getHeading());

        secondLineupPath = new Path (new BezierLine(Shooting,secondLineup));
        secondLineupPath.setConstantHeadingInterpolation(secondLineup.getHeading());

        secondPickupPath = new Path (new BezierLine(secondLineup,secondPickup));
        secondPickupPath.setConstantHeadingInterpolation(secondPickup.getHeading());

        secondPickupBackPath = new Path (new BezierLine(secondPickup,secondPickupBack));
        secondPickupBackPath.setConstantHeadingInterpolation(secondPickupBack.getHeading());

        shootSecondLineupPath = new Path (new BezierLine(secondPickupBack,movingOffLine));
        shootSecondLineupPath.setConstantHeadingInterpolation(movingOffLine.getHeading());

        //movingBackPath = new Path (new BezierLine(Shooting, movingOffLine));
        //movingBackPath.setConstantHeadingInterpolation(movingOffLine.getHeading());

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:

                if(!follower.isBusy()) {

                    robot.intake.setPower(1);
                    robot.L_swingythingy.setPosition(robot.L_swingy_Thingy_Close);
                    robot.R_swingythingy.setPosition(robot.R_swingy_Thingy_Close);

                    follower.followPath(scorePreload, true);
                    setPathState(1);
                }

                break;

            case 1:

                if(!follower.isBusy()) {

                    robot.hoodDown();

                    if(!robot.timerInitted[4]) {//very very first thing to happen
                        robot.timeArray[4] = robot.currentTime.milliseconds();
                        robot.timerInitted[4] = true;
                    }

                    if (robot.currentTime.milliseconds() > robot.timeArray[4] + 6100) {//Last thing to happen
                        setPathState(2);
                        robot.timerInitted[4] = false;
                    }

                    else if (robot.currentTime.milliseconds() > robot.timeArray[4] + 6000) {//Last thing to happen

                        robot.shoot();
                    }

                    else if (robot.currentTime.milliseconds() > robot.timeArray[4] + 3000) {

                        robot.shoot();
                    }

                    else {//Second thing to happen
                        robot.targetRPM = 3500;
                        robot.shoot();

                    }
                }
                break;
            case 2:

                if(!follower.isBusy()) {

                    robot.intake();
                    follower.followPath(firstLineupPath, true);
                    setPathState(3);
                }
                break;
            case 3:

                if(!follower.isBusy()) {

                    follower.followPath(firstPickupPath, true);
                    setPathState(4);
                }
                break;
            case 4:

                if(!follower.isBusy()) {

                    follower.followPath(shootFirstPickupPath, true);
                    setPathState(5);
                }
                break;
            case 5:

                if(!follower.isBusy()) {

                    robot.hoodDown();

                    if(!robot.timerInitted[5]) {//very very first thing to happen
                        robot.timeArray[5] = robot.currentTime.milliseconds();
                        robot.timerInitted[5] = true;
                    }

                    if (robot.currentTime.milliseconds() > robot.timeArray[5] + 5100) {//Last thing to happen
                        setPathState(6);
                        robot.timerInitted[5] = false;
                    }

                    else if (robot.currentTime.milliseconds() > robot.timeArray[5] + 5000) {//Last thing to happen

                        robot.shoot();
                    }

                    else if (robot.currentTime.milliseconds() > robot.timeArray[5] + 2500) {

                        robot.shoot();
                    }

                    else {//Second thing to happen
                        robot.targetRPM = 3500;
                        robot.shoot();

                    }


                }
                break;
            case 6:

                if(!follower.isBusy()) {
                    robot.intake();
                    follower.followPath(secondLineupPath, true);
                    setPathState(7);
                }
                break;
            case 7:

                if(!follower.isBusy()) {
                    follower.followPath(secondPickupPath, true);
                    setPathState(8);
                }
                break;
            case 8:

                if(!follower.isBusy()) {

                    follower.followPath(secondPickupBackPath, true);
                    setPathState(9);
                }
                break;
            case 9:

                if(!follower.isBusy()) {
                    follower.followPath(shootSecondLineupPath, true);
                    setPathState(10);
                }
                break;
            case 10:

                if(!follower.isBusy()) {
                    robot.hoodDown();

                    if(!robot.timerInitted[6]) {//very very first thing to happen
                        robot.timeArray[6] = robot.currentTime.milliseconds();
                        robot.timerInitted[6] = true;
                    }

                                    if (robot.currentTime.milliseconds() > robot.timeArray[6] + 5100) {//Last thing to happen
                                        setPathState(-1);
                                        robot.timerInitted[6] = false;
                                    }

                                else if (robot.currentTime.milliseconds() > robot.timeArray[6] + 5000) {//Last thing to happen

                                    robot.shoot();
                                }

                            else if (robot.currentTime.milliseconds() > robot.timeArray[6] + 2500) {

                                robot.shoot();
                            }

                        else {//Second thing to happen
                            robot.targetRPM = 3500;
                            robot.shoot();

                        }
                }
                break;
            case 11:

                if(!follower.isBusy()) {
                    robot.stopIntake();
                    follower.followPath(movingBackPath, true);
                    setPathState(-1);
                }
                break;

        }
    }

    public void blockingSleep(int milliseconds){
        ElapsedTime timer = new ElapsedTime();

        timer.reset();
        int targetTime = (int)timer.milliseconds() + milliseconds;

        while(timer.milliseconds() < targetTime){

        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {

        follower.update();
        autonomousPathUpdate();

        robot.R_shooter.setPower(robot.setting_ShooterRPM());
        robot.L_shooter.setPower(robot.setting_ShooterRPM());

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        robot = new Gen2Hardwaremap(hardwareMap);

        robot.L_swingythingy.setPosition(robot.L_swingy_Thingy_Close);
        robot.R_swingythingy.setPosition(robot.R_swingy_Thingy_Close);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {
    }
}