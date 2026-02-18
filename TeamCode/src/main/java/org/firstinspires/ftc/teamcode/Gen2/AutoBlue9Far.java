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
@Autonomous(name = "AutoBlue9Far")
public class AutoBlue9Far extends OpMode {

    Gen2Hardwaremap robot = null;
    private Follower follower;
    private Timer pathTimer,  opmodeTimer;
    private int pathState;
    private final Pose startPose = new Pose(49, 0, Math.toRadians(90));
    private final Pose Shooting = new Pose(36, 93, Math.toRadians(135));
    private final Pose firstLineup = new Pose(40, 75, Math.toRadians(180) );
    private final Pose firstPickup = new Pose(11, 75, Math.toRadians(180) );
    private final Pose secondLineup = new Pose(40, 50, Math.toRadians(180));
    private final Pose secondPickup = new Pose(3, 53, Math.toRadians(180));
    private final Pose secondPickupBack = new Pose(30, 53, Math.toRadians(180));
    private final Pose movingOffLine = new Pose(90, 102, Math.toRadians(45) );

    private Path scorePreload, firstLineupPath, firstPickupPath, shootFirstPickupPath, secondLineupPath, secondPickupPath, secondPickupBackPath, shootSecondLineupPath, movingBackPath;

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

        shootSecondLineupPath = new Path (new BezierLine(secondPickupBack,Shooting));
        shootSecondLineupPath.setConstantHeadingInterpolation(Shooting.getHeading());

        movingBackPath = new Path(new BezierLine(Shooting, movingOffLine));
        movingBackPath.setConstantHeadingInterpolation(Shooting.getHeading());

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:

                if(!follower.isBusy()) {

                    follower.followPath(scorePreload, true);
                    setPathState(2);
                }

                break;
            case 2:

                if(!follower.isBusy()) {

                    robot.hoodDown();
                    robot.autoShoot();
                    setPathState(3);
                }
                break;
            case 3:

                if(!follower.isBusy()) {
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

                    setPathState(7);
                }
                break;
            case 7:

                if(!follower.isBusy()) {
                    follower.followPath(secondLineupPath, true);
                    setPathState(8);
                }
                break;
            case 8:

                if(!follower.isBusy()) {

                    follower.followPath(secondPickupPath, true);
                    setPathState(9);
                }
                break;
            case 9:

                if(!follower.isBusy()) {

                    follower.followPath(secondPickupBackPath, true);
                    setPathState(10);
                }
                break;
            case 10:

                if(!follower.isBusy()) {
                    follower.followPath(shootSecondLineupPath, true);
                    setPathState(11);
                }
                break;
            case 11:

                if(!follower.isBusy()) {

                    setPathState(12);
                }
                break;
            case 12:

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

        follower.update();
        autonomousPathUpdate();

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