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
@Autonomous(name = "AutoBlue6Close")
public class AutoBlue6Close extends OpMode {

    Gen2Hardwaremap robot = null;
    private Follower follower;
    private Timer pathTimer,  opmodeTimer;

    private int pathState;

    private final Pose startPose = new Pose(31, 114, Math.toRadians(135));
    private final Pose Shooting = new Pose(54, 92, Math.toRadians(135));
    private final Pose firstLineup = new Pose(60, 77, Math.toRadians(180));
    private final Pose firstPickup = new Pose(34, 77, Math.toRadians(180));
    private final Pose movingOffLine = new Pose(60, 108, Math.toRadians(150));

    private Path scorePreload, firstLineupPath, firstPickupPath, shootFirstPickupPath, movingBackPath;

    public void buildPaths() {


        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, Shooting));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), Shooting.getHeading());

        firstLineupPath = new Path (new BezierLine(Shooting,firstLineup));
        firstLineupPath.setConstantHeadingInterpolation(firstLineup.getHeading());

        firstPickupPath = new Path (new BezierLine(firstLineup,firstPickup));
        firstPickupPath.setConstantHeadingInterpolation(firstPickup.getHeading());

        shootFirstPickupPath = new Path (new BezierLine(firstPickup,Shooting));
        shootFirstPickupPath.setConstantHeadingInterpolation(Shooting.getHeading());

        movingBackPath = new Path(new BezierLine(Shooting, movingOffLine));
        movingBackPath.setConstantHeadingInterpolation(Shooting.getHeading());

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

                    if(!robot.timerInitted[11]) {//very very first thing to happen
                        robot.timeArray[11] = robot.currentTime.milliseconds();
                        robot.timerInitted[11] = true;
                    }

                    if (robot.currentTime.milliseconds() > robot.timeArray[11] + 6100) {//Last thing to happen
                        setPathState(2);
                        robot.timerInitted[11] = false;
                    }

                    else if (robot.currentTime.milliseconds() > robot.timeArray[11] + 6000) {//Last thing to happen

                        robot.autoShoot();
                    }

                    else if (robot.currentTime.milliseconds() > robot.timeArray[11] + 3000) {

                        robot.autoShoot();
                    }

                    else {//Second thing to happen
                        robot.targetRPM = 3500;
                        robot.autoShoot();

                    }

                    setPathState(3);
                }
                break;
            case 3:

                if(!follower.isBusy()) {

                    robot.intake();
                    follower.followPath(firstLineupPath, true);
                    setPathState(4);
                }
                break;
            case 4:

                if(!follower.isBusy()) {

                    follower.followPath(firstPickupPath, true);
                    setPathState(5);
                }
                break;
            case 5:

                if(!follower.isBusy()) {
                    follower.followPath(shootFirstPickupPath, true);
                    setPathState(6);
                }
                break;
            case 6:

                if(!follower.isBusy()) {
                    robot.hoodDown();

                    if(!robot.timerInitted[12]) {//very very first thing to happen
                        robot.timeArray[12] = robot.currentTime.milliseconds();
                        robot.timerInitted[12] = true;
                    }

                    if (robot.currentTime.milliseconds() > robot.timeArray[12] + 5100) {//Last thing to happen
                        setPathState(6);
                        robot.timerInitted[12] = false;
                    }

                    else if (robot.currentTime.milliseconds() > robot.timeArray[12] + 5000) {//Last thing to happen

                        robot.autoShoot();
                    }

                    else if (robot.currentTime.milliseconds() > robot.timeArray[12] + 2500) {

                        robot.autoShoot();
                    }

                    else {//Second thing to happen
                        robot.targetRPM = 3500;
                        robot.autoShoot();

                    }
                    setPathState(7);
                }
                break;
            case 7:

                if(!follower.isBusy()) {

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

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
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