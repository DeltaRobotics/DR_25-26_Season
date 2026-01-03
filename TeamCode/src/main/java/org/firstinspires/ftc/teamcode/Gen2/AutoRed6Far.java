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

import org.firstinspires.ftc.teamcode.Gen2.Gen2Hardwaremap;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Disabled
@Autonomous(name = "AutoRed6Far")
public class AutoRed6Far extends OpMode {

    Gen2Hardwaremap robot = null;
    private Follower follower;
    private Timer pathTimer,  opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(50, 0, Math.toRadians(90));
    private final Pose Shooting = new Pose(48, 79, Math.toRadians(135));
    private final Pose firstLineup = new Pose(45, 75, Math.toRadians(180) );
    private final Pose firstPickup = new Pose(16, 75, Math.toRadians(180) );
    private final Pose movingBack = new Pose(39, 66, Math.toRadians(135));

    private Path scorePreload, firstLineupPath, firstPickupPath, shootFirstPickupPath, movingBackPath;

    public void buildPaths() {

        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), Shooting.getHeading());

        firstLineupPath = new Path (new BezierLine(Shooting,firstLineup));
        firstLineupPath.setConstantHeadingInterpolation(firstLineup.getHeading());

        firstPickupPath = new Path (new BezierLine(firstLineup,firstPickup));
        firstPickupPath.setConstantHeadingInterpolation(firstPickup.getHeading());

        shootFirstPickupPath = new Path (new BezierLine(firstPickup,Shooting));
        shootFirstPickupPath.setConstantHeadingInterpolation(Shooting.getHeading());

        movingBackPath = new Path(new BezierLine(Shooting, movingBack));
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