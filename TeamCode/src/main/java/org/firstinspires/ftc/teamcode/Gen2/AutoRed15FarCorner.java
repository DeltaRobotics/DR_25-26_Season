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
@Autonomous(name = "AutoRed15FarCorner")
public class AutoRed15FarCorner extends OpMode {

    Gen2Hardwaremap robot = null;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private final Pose startPose = new Pose(80, 0, Math.toRadians(90));
    private final Pose Shooting = new Pose(83, 10, Math.toRadians(69));
    private final Pose pickup = new Pose(132, 10, Math.toRadians(0));
    private final Pose pickupMove = new Pose(130, 14, Math.toRadians(0));
    private final Pose movingOffLine = new Pose(100, 10, Math.toRadians(90));
    private Path scorePreload, pickupPath,pickupMovePath, shootPickupPath, movingOffLinePath ;

    public void buildPaths() {

        scorePreload = new Path(new BezierLine(startPose, Shooting));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), Shooting.getHeading()); //startPose.getHeading(), Shooting.getHeading()



        pickupPath = new Path (new BezierLine(Shooting,pickup));
        pickupPath.setConstantHeadingInterpolation( pickup.getHeading());//Shooting.getHeading(),

        pickupMovePath = new Path (new BezierLine(pickup,pickupMove));
        pickupMovePath.setLinearHeadingInterpolation( pickup.getHeading(), pickupMove.getHeading());//Shooting.getHeading(),

        shootPickupPath = new Path (new BezierLine(pickupMove,Shooting));
        shootPickupPath.setLinearHeadingInterpolation(pickupMove.getHeading(), Shooting.getHeading());

        movingOffLinePath = new Path(new BezierLine(Shooting, movingOffLine));
        movingOffLinePath.setLinearHeadingInterpolation(Shooting.getHeading(), movingOffLine.getHeading());//Shooting.getHeading()


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

                    if(!robot.timerInitted[4]) {//very very first thing to happen
                        robot.timeArray[4] = robot.currentTime.milliseconds();
                        robot.timerInitted[4] = true;
                    }

                    if (robot.currentTime.milliseconds() > robot.timeArray[4] + 1800) {//Last thing to happen
                        setPathState(2);
                        robot.timerInitted[4] = false;
                    }

                    else {//Second thing to happen
                        robot.autoShoot();
                        robot.display_state_shooting();

                    }
                }
                break;
            case 2:

                if(!follower.isBusy()) {

                    robot.intake();
                    follower.followPath(pickupPath, true);
                    robot.display_state_intaking();
                    setPathState(3);
                }
                break;
            case 3:

                if(!follower.isBusy()) {

                    follower.followPath(pickupMovePath, true);
                    setPathState(4);
                }
                break;
            case 4:

                if(!follower.isBusy()) {

                    follower.followPath(shootPickupPath, true);
                    setPathState(5);
                }
                break;
            case 5:

                if(!follower.isBusy()) {

                    if(!robot.timerInitted[5]) {//very very first thing to happen
                        robot.timeArray[5] = robot.currentTime.milliseconds();
                        robot.timerInitted[5] = true;
                    }

                    if (robot.currentTime.milliseconds() > robot.timeArray[5] + 1800) {//Last thing to happen
                        setPathState(6);
                        robot.timerInitted[5] = false;
                    }

                    else {//Second thing to happen
                        robot.autoShoot();
                        robot.display_state_shooting();

                    }
                }
                break;
            case 6:

                if(!follower.isBusy()) {
                    robot.intake();
                    follower.followPath(pickupPath, true);
                    robot.display_state_RedIdle();
                    setPathState(7);
                }
                break;
            case 7:

                if(!follower.isBusy()) {
                    follower.followPath(pickupMovePath, true);
                    setPathState(9);
                }
                break;
            case 9:

                if(!follower.isBusy()) {
                    follower.followPath(shootPickupPath, true);
                    setPathState(10);
                }
                break;
            case 10:

                if(!follower.isBusy()) {

                    if(!robot.timerInitted[5]) {//very very first thing to happen
                        robot.timeArray[5] = robot.currentTime.milliseconds();
                        robot.timerInitted[5] = true;
                    }

                    if (robot.currentTime.milliseconds() > robot.timeArray[5] + 1800) {//Last thing to happen
                        setPathState(11);
                        robot.timerInitted[5] = false;
                    }

                    else {//Second thing to happen
                        robot.autoShoot();
                        robot.display_state_shooting();

                    }
                }
                break;
            case 11:

                if(!follower.isBusy()) {
                    robot.intake();
                    follower.followPath(pickupPath, true);
                    robot.display_state_RedIdle();
                    setPathState(12);
                }
                break;
            case 12:

                if(!follower.isBusy()) {
                    follower.followPath(pickupMovePath, true);
                    setPathState(13);
                }
                break;
            case 13:

                if(!follower.isBusy()) {
                    follower.followPath(shootPickupPath, true);
                    setPathState(14);
                }
                break;
            case 14:

                if(!follower.isBusy()) {

                    if(!robot.timerInitted[5]) {//very very first thing to happen
                        robot.timeArray[5] = robot.currentTime.milliseconds();
                        robot.timerInitted[5] = true;
                    }

                    if (robot.currentTime.milliseconds() > robot.timeArray[5] + 1800) {//Last thing to happen
                        setPathState(15);
                        robot.timerInitted[5] = false;
                    }

                    else {//Second thing to happen
                        robot.autoShoot();
                        robot.display_state_shooting();

                    }
                }
                break;
            case 15:

                if(!follower.isBusy()) {
                    robot.intake();
                    follower.followPath(pickupPath, true);
                    robot.display_state_RedIdle();
                    setPathState(16);
                }
                break;
            case 16:

                if(!follower.isBusy()) {
                    follower.followPath(pickupMovePath, true);
                    setPathState(17);
                }
                break;
            case 17:

                if(!follower.isBusy()) {
                    follower.followPath(shootPickupPath, true);
                    setPathState(18);
                }
                break;
            case 18:

                if(!follower.isBusy()) {

                    if(!robot.timerInitted[5]) {//very very first thing to happen
                        robot.timeArray[5] = robot.currentTime.milliseconds();
                        robot.timerInitted[5] = true;
                    }

                    if (robot.currentTime.milliseconds() > robot.timeArray[5] + 1800) {//Last thing to happen
                        setPathState(19);
                        robot.timerInitted[5] = false;
                    }

                    else {//Second thing to happen
                        robot.autoShoot();
                        robot.display_state_shooting();

                    }
                }
                break;
            case 19:

                if(!follower.isBusy()) {
                    robot.stopIntake();
                    follower.followPath(movingOffLinePath, true);
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
        robot.turret(telemetry);

        robot.R_shooter.setPower(robot.setting_ShooterRPM());
        robot.L_shooter.setPower(robot.setting_ShooterRPM());

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("tx", robot.limelight.getLatestResult().getFiducialResults().isEmpty() ? "No Target" : robot.limelight.getLatestResult().getFiducialResults().get(0).getTargetXDegrees());
        telemetry.update();;
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        robot = new Gen2Hardwaremap(hardwareMap);

        robot.L_swingythingy.setPosition(robot.L_swingy_Thingy_Close);
        robot.R_swingythingy.setPosition(robot.R_swingy_Thingy_Close);

        robot.transfer.setPower(0);

        robot.L_PTO.setPosition(robot.L_PTO_UP);
        robot.R_PTO.setPosition(robot.R_PTO_UP);

        robot.R_feeder.setPower(0);
        robot.L_feeder.setPower(0);

        robot.R_turret.setPower(0);
        robot.L_turret.setPower(0);

        robot.targetRPM = 1000;

        robot.autoFarShoot();

        robot.blue = false;

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        //robot.turret(telemetry);
        setPathState(0);
    }

    @Override
    public void stop() {
    }
}