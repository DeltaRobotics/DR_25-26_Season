package org.firstinspires.ftc.teamcode.Gen2;

import android.graphics.Point;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.FuturePose;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


//@Disabled
@Autonomous(name = "bezierCurveTest")
public class AutoRed3Far extends OpMode {
    private Follower follower;

    Gen2Hardwaremap robot = null;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;
    private final Pose startPose = new Pose(78, 0);

    private final Pose shootingPose = new Pose(80, 79, Math.toRadians(45));

    private final Pose controlPickup = new Pose (77, 59);

    private final Pose pickup = new Pose(108,59, Math.toRadians(0));

    private final Pose movingBack = new Pose(90, 66, Math.toRadians(45));

    private PathChain movingBackPath,  shooting, testPath;
    private Path startPath;


    public void buildPaths() {

        startPath = new Path (new BezierLine(startPose,shootingPose));
        startPath.setConstantHeadingInterpolation(shootingPose.getHeading());

         //testPath = follower.pathBuilder()
                //.addPath(new BezierCurve(shootingPose, controlPickup, pickup)).build()
                //.setHeadingInterpolator(pickup.getHeading())
                //.build();
    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:

                if(!follower.isBusy()) {

                    robot.hoodUp();
                    robot.autoShoot();

                    follower.followPath(startPath, true);
                    setPathState(1);
                }

                break;
            case 1:

                if(!follower.isBusy()) {

                    follower.followPath(movingBackPath, true);

                    setPathState(-1);
                }
                break;

            case 2:

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