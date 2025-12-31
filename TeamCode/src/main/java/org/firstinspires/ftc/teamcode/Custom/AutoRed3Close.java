package org.firstinspires.ftc.teamcode.Custom;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous(name = "AutoRed3Close")
//@Disabled
public class AutoRed3Close extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    Gen2Hardwaremap robot = null;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(120, 110, Math.toRadians(45));

    private final Pose Shooting = new Pose(100, 90, Math.toRadians(45));

    private final Pose movingOffLine = new Pose(90, 102, Math.toRadians(45) );

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */



    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload, movingBackPath;


    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        scorePreload = new Path(new BezierLine(startPose, Shooting));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), Shooting.getHeading());

        movingBackPath = new Path (new BezierLine(Shooting, movingOffLine));
        movingBackPath.setConstantHeadingInterpolation(Shooting.getHeading());

    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:

                if(!follower.isBusy()) {

                    robot.closeShooting();

                    if(robot.waiting){
                        setPathState(1);
                    }

                    follower.followPath(scorePreload, true);

                }

                break;
            case 1:

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


    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        if(robot.timerInitted){
            robot.closeShooting();
        }

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
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

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}