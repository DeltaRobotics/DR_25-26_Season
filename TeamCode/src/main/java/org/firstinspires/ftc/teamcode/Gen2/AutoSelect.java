package org.firstinspires.ftc.teamcode.Gen2;


import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.telemetry.SelectableOpMode;
import com.pedropathing.util.PoseHistory;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;

//@Disabled
@Autonomous(name = "AutoSelect")
public class AutoSelect extends SelectableOpMode {

    public static Follower follower;


    @IgnoreConfigurable
    static PoseHistory poseHistory;

    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    @IgnoreConfigurable
    static ArrayList<String> changes = new ArrayList<>();

/** Selection **/
    public AutoSelect() {
        super("Select an Auto Configuration", s -> {
            s.folder("Blue", z -> {
                z.folder("Close", l -> {
                    l.add("9", Blue9Close::new);
                    l.add("6", Blue6Close::new);
                });
                z.folder("Far", a -> {
                    a.add("9", Blue9Far::new);
                    a.add("6", Blue6Far::new);
                });
            });
            s.folder("Red", z -> {
                z.folder("Close", l -> {
                    l.add("9", Red9Close::new);
                    l.add("6", Red6Close::new);
                });
                z.folder("Far", a -> {
                    a.add("9", Red9Far::new);
                    a.add("6", Red6Far::new);
                });
            });

        });
    }


}
/** Actual Auto Code **/



/** Red 9 **/
class Red9Close extends OpMode {
    //@Override

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
                        robot.R_shooter.setPower(robot.setting_ShooterRPM());
                        robot.L_shooter.setPower(robot.setting_ShooterRPM());
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
                        robot.R_shooter.setPower(robot.setting_ShooterRPM());
                        robot.L_shooter.setPower(robot.setting_ShooterRPM());
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
                        robot.shoot();

                    }
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

        robot.turret(true, 1, true, 3500);

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

        robot.initAprilTag(hardwareMap);

        robot.blue = false;

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
class Red9Far extends OpMode {
    //@Override

    Gen2Hardwaremap robot = null;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private final Pose startPose = new Pose(80, 0, Math.toRadians(90));
    private final Pose Shooting = new Pose(100, 90, Math.toRadians(45));
    private final Pose firstLineup = new Pose(85, 79, Math.toRadians(0));
    private final Pose firstPickup = new Pose(110, 80, Math.toRadians(0));
    private final Pose secondLineup = new Pose(85, 54, Math.toRadians(0));
    private final Pose secondPickup = new Pose(120, 55, Math.toRadians(0));
    private final Pose secondPickupBack = new Pose(90, 55, Math.toRadians(0));
    private final Pose movingOffLine = new Pose(90, 102, Math.toRadians(45));

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

        shootSecondLineupPath = new Path (new BezierLine(secondPickup,Shooting));
        shootSecondLineupPath.setConstantHeadingInterpolation(Shooting.getHeading());

        movingBackPath = new Path (new BezierLine(Shooting,movingOffLine));
        movingBackPath.setConstantHeadingInterpolation(movingOffLine.getHeading());


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
                        robot.R_shooter.setPower(robot.setting_ShooterRPM());
                        robot.L_shooter.setPower(robot.setting_ShooterRPM());
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
                        robot.R_shooter.setPower(robot.setting_ShooterRPM());
                        robot.L_shooter.setPower(robot.setting_ShooterRPM());
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
                        robot.shoot();

                    }
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

        robot.turret(true, 1, true, 6000);

        robot.R_shooter.setPower(robot.setting_ShooterRPM());
        robot.L_shooter.setPower(robot.setting_ShooterRPM());

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

        robot.initAprilTag(hardwareMap);

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
        setPathState(0);
    }

    @Override
    public void stop() {
    }
}



/** Red 6 **/
class Red6Close extends OpMode {
    //@Override

    Gen2Hardwaremap robot = null;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(113, 114, Math.toRadians(45));
    private final Pose Shooting = new Pose(90, 92, Math.toRadians(45));
    private final Pose firstLineup = new Pose(84, 77, Math.toRadians(0));
    private final Pose firstPickup = new Pose(110, 77, Math.toRadians(0));
    private final Pose movingOffLine = new Pose(84, 108, Math.toRadians(30));


    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */


    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload, firstLineupPath, firstPickupPath, shootFirstPickupPath, movingBackPath ;


    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        scorePreload = new Path(new BezierLine(startPose, Shooting));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), Shooting.getHeading());

        firstLineupPath = new Path (new BezierLine(Shooting,firstLineup));
        firstLineupPath.setConstantHeadingInterpolation(firstLineup.getHeading());

        firstPickupPath = new Path (new BezierLine(firstLineup,firstPickup));
        firstPickupPath.setConstantHeadingInterpolation(firstPickup.getHeading());

        shootFirstPickupPath = new Path (new BezierLine(firstPickup,movingOffLine));
        shootFirstPickupPath.setConstantHeadingInterpolation(movingOffLine.getHeading());

        //movingBackPath = new Path (new BezierLine(Shooting,movingOffLine));
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

                    if(!robot.timerInitted[10]) {//very very first thing to happen
                        robot.timeArray[10] = robot.currentTime.milliseconds();
                        robot.timerInitted[10] = true;
                    }

                    if (robot.currentTime.milliseconds() > robot.timeArray[10] + 6100) {//Last thing to happen
                        setPathState(2);
                        robot.timerInitted[10] = false;
                    }

                    else if (robot.currentTime.milliseconds() > robot.timeArray[10] + 6000) {//Last thing to happen

                        robot.shoot();
                    }

                    else if (robot.currentTime.milliseconds() > robot.timeArray[10] + 3000) {

                        robot.shoot();
                    }

                    else {//Second thing to happen
                        robot.R_shooter.setPower(robot.setting_ShooterRPM());
                        robot.L_shooter.setPower(robot.setting_ShooterRPM());
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

                    if(!robot.timerInitted[11]) {//very very first thing to happen
                        robot.timeArray[11] = robot.currentTime.milliseconds();
                        robot.timerInitted[11] = true;
                    }

                    if (robot.currentTime.milliseconds() > robot.timeArray[11] + 5100) {//Last thing to happen
                        setPathState(-1);
                        robot.timerInitted[11] = false;
                    }

                    else if (robot.currentTime.milliseconds() > robot.timeArray[11] + 5000) {//Last thing to happen

                        robot.shoot();
                    }

                    else if (robot.currentTime.milliseconds() > robot.timeArray[11] + 2500) {

                        robot.shoot();
                    }

                    else {//Second thing to happen
                        robot.R_shooter.setPower(robot.setting_ShooterRPM());
                        robot.L_shooter.setPower(robot.setting_ShooterRPM());
                        robot.shoot();

                    }

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

        robot.turret(true, 1, true, 3500);

        robot.R_shooter.setPower(robot.setting_ShooterRPM());
        robot.L_shooter.setPower(robot.setting_ShooterRPM());

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

        robot.initAprilTag(hardwareMap);

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
        setPathState(0);
    }

    @Override
    public void stop() {
    }
}

class Red6Far extends OpMode {
    //@Override

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

        robot.turret(true, 1, true, 6000);

        robot.R_shooter.setPower(robot.setting_ShooterRPM());
        robot.L_shooter.setPower(robot.setting_ShooterRPM());

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

        robot.initAprilTag(hardwareMap);

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
        setPathState(0);
    }

    @Override
    public void stop() {
    }
}



/** Blue 9 **/
class Blue9Close extends OpMode {
    Gen2Hardwaremap robot = null;
    private Follower follower;
    private Timer pathTimer,  opmodeTimer;

    private int pathState;

    private final Pose startPose = new Pose(31, 114, Math.toRadians(135));
    private final Pose Shooting = new Pose(54, 92, Math.toRadians(135));
    private final Pose firstLineup = new Pose(60, 77, Math.toRadians(180));
    private final Pose firstPickup = new Pose(34, 77, Math.toRadians(180));
    private final Pose secondLineup = new Pose(56, 53, Math.toRadians(180));
    private final Pose secondPickup = new Pose(26, 53, Math.toRadians(180));
    private final Pose secondPickupBack = new Pose(54, 52, Math.toRadians(180));
    private final Pose movingOffLine = new Pose(60, 108, Math.toRadians(150));

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

        shootSecondLineupPath = new Path (new BezierLine(secondPickupBack,movingOffLine));
        shootSecondLineupPath.setConstantHeadingInterpolation(movingOffLine.getHeading());

        //movingBackPath = new Path(new BezierLine(Shooting, movingOffLine));
        //movingBackPath.setConstantHeadingInterpolation(Shooting.getHeading());

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

                    if(!robot.timerInitted[8]) {//very very first thing to happen
                        robot.timeArray[8] = robot.currentTime.milliseconds();
                        robot.timerInitted[8] = true;
                    }

                    if (robot.currentTime.milliseconds() > robot.timeArray[8] + 6100) {//Last thing to happen
                        setPathState(2);
                        robot.timerInitted[8] = false;
                    }

                    else if (robot.currentTime.milliseconds() > robot.timeArray[8] + 6000) {//Last thing to happen

                        robot.shoot();
                    }

                    else if (robot.currentTime.milliseconds() > robot.timeArray[8] + 3000) {

                        robot.shoot();
                    }

                    else {//Second thing to happen
                        robot.R_shooter.setPower(robot.setting_ShooterRPM());
                        robot.L_shooter.setPower(robot.setting_ShooterRPM());
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

                    if(!robot.timerInitted[9]) {//very very first thing to happen
                        robot.timeArray[9] = robot.currentTime.milliseconds();
                        robot.timerInitted[9] = true;
                    }

                    if (robot.currentTime.milliseconds() > robot.timeArray[9] + 5100) {//Last thing to happen
                        setPathState(6);
                        robot.timerInitted[9] = false;
                    }

                    else if (robot.currentTime.milliseconds() > robot.timeArray[9] + 5000) {//Last thing to happen

                        robot.shoot();
                    }

                    else if (robot.currentTime.milliseconds() > robot.timeArray[9] + 2500) {

                        robot.shoot();
                    }

                    else {//Second thing to happen
                        robot.R_shooter.setPower(robot.setting_ShooterRPM());
                        robot.L_shooter.setPower(robot.setting_ShooterRPM());
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

                    if(!robot.timerInitted[10]) {//very very first thing to happen
                        robot.timeArray[10] = robot.currentTime.milliseconds();
                        robot.timerInitted[10] = true;
                    }

                    if (robot.currentTime.milliseconds() > robot.timeArray[10] + 5100) {//Last thing to happen
                        setPathState(-1);
                        robot.timerInitted[10] = false;
                    }

                    else if (robot.currentTime.milliseconds() > robot.timeArray[10] + 5000) {//Last thing to happen

                        robot.shoot();
                    }

                    else if (robot.currentTime.milliseconds() > robot.timeArray[10] + 2500) {

                        robot.shoot();
                    }

                    else {//Second thing to happen
                        robot.shoot();

                    }
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

        robot.turret(true, 1, true, 3500);

        robot.R_shooter.setPower(robot.setting_ShooterRPM());
        robot.L_shooter.setPower(robot.setting_ShooterRPM());

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

        robot.initAprilTag(hardwareMap);

        robot.blue = true;

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
class Blue9Far extends OpMode {
    Gen2Hardwaremap robot = null;
    private Follower follower;
    private Timer pathTimer,  opmodeTimer;
    private int pathState;
    private final Pose startPose = new Pose(48, 0, Math.toRadians(90));
    private final Pose Shooting = new Pose(50, 2, Math.toRadians(100));
    private final Pose firstLineup = new Pose(36, 27, Math.toRadians(180));
    private final Pose firstPickup = new Pose(6, 27, Math.toRadians(180));
    private final Pose secondLineup = new Pose(33, 51, Math.toRadians(180));
    private final Pose secondPickup = new Pose(3, 49, Math.toRadians(180));
    private final Pose movingOffLine = new Pose(50, 23, Math.toRadians(90));

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

        shootSecondLineupPath = new Path (new BezierLine(secondPickup,Shooting));
        shootSecondLineupPath.setConstantHeadingInterpolation(Shooting.getHeading());

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

                    if(!robot.timerInitted[8]) {//very very first thing to happen
                        robot.timeArray[8] = robot.currentTime.milliseconds();
                        robot.timerInitted[8] = true;
                    }

                    if (robot.currentTime.milliseconds() > robot.timeArray[8] + 6100) {//Last thing to happen
                        setPathState(2);
                        robot.timerInitted[8] = false;
                    }

                    else if (robot.currentTime.milliseconds() > robot.timeArray[8] + 6000) {//Last thing to happen

                        robot.shoot();
                    }

                    else if (robot.currentTime.milliseconds() > robot.timeArray[8] + 3000) {

                        robot.shoot();
                    }

                    else {//Second thing to happen
                        robot.R_shooter.setPower(robot.setting_ShooterRPM());
                        robot.L_shooter.setPower(robot.setting_ShooterRPM());
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

                    if(!robot.timerInitted[9]) {//very very first thing to happen
                        robot.timeArray[9] = robot.currentTime.milliseconds();
                        robot.timerInitted[9] = true;
                    }

                    if (robot.currentTime.milliseconds() > robot.timeArray[9] + 5100) {//Last thing to happen
                        setPathState(6);
                        robot.timerInitted[9] = false;
                    }

                    else if (robot.currentTime.milliseconds() > robot.timeArray[9] + 5000) {//Last thing to happen

                        robot.shoot();
                    }

                    else if (robot.currentTime.milliseconds() > robot.timeArray[9] + 2500) {

                        robot.shoot();
                    }

                    else {//Second thing to happen
                        robot.R_shooter.setPower(robot.setting_ShooterRPM());
                        robot.L_shooter.setPower(robot.setting_ShooterRPM());
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

                if(!follower.isBusy()){
                follower.followPath(shootSecondLineupPath, true);
                setPathState(9);
            }
                break;
            case 9:

                if(!follower.isBusy()) {
                    robot.hoodDown();

                    if(!robot.timerInitted[10]) {//very very first thing to happen
                        robot.timeArray[10] = robot.currentTime.milliseconds();
                        robot.timerInitted[10] = true;
                    }

                    if (robot.currentTime.milliseconds() > robot.timeArray[10] + 5100) {//Last thing to happen
                        setPathState(-1);
                        robot.timerInitted[10] = false;
                    }

                    else if (robot.currentTime.milliseconds() > robot.timeArray[10] + 5000) {//Last thing to happen

                        robot.shoot();
                    }

                    else if (robot.currentTime.milliseconds() > robot.timeArray[10] + 2500) {

                        robot.shoot();
                    }

                    else {//Second thing to happen
                        robot.shoot();

                    }
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

        robot.turret(true, 1, true, 6000);

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

        robot.initAprilTag(hardwareMap);

        robot.blue = true;

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



/** Blue 6 **/
class Blue6Close extends OpMode {

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

                        robot.shoot();
                    }

                    else if (robot.currentTime.milliseconds() > robot.timeArray[11] + 3000) {

                        robot.shoot();
                    }

                    else {//Second thing to happen
                        robot.R_shooter.setPower(robot.setting_ShooterRPM());
                        robot.L_shooter.setPower(robot.setting_ShooterRPM());
                        robot.shoot();
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

                        robot.shoot();
                    }

                    else if (robot.currentTime.milliseconds() > robot.timeArray[12] + 2500) {

                        robot.shoot();
                    }

                    else {//Second thing to happen
                        robot.R_shooter.setPower(robot.setting_ShooterRPM());
                        robot.L_shooter.setPower(robot.setting_ShooterRPM());
                        robot.shoot();
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

        robot.turret(true, 1, true, 3500);

        robot.R_shooter.setPower(robot.setting_ShooterRPM());
        robot.L_shooter.setPower(robot.setting_ShooterRPM());

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

        robot.initAprilTag(hardwareMap);

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
        setPathState(0);
    }

    @Override
    public void stop() {
    }

}

class Blue6Far extends OpMode {
    Gen2Hardwaremap robot = null;
    private Follower follower;
    private Timer pathTimer,  opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(48, 0, Math.toRadians(90));
    private final Pose Shooting = new Pose(50, 2, Math.toRadians(100));
    private final Pose firstLineup = new Pose(36, 27, Math.toRadians(180));
    private final Pose firstPickup = new Pose(6, 27, Math.toRadians(180));
    private final Pose movingOffLine = new Pose(50, 23, Math.toRadians(90));

    private Path scorePreload, firstLineupPath, firstPickupPath, shootFirstPickupPath, movingOffLinePath;

    public void buildPaths() {

        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), Shooting.getHeading());

        firstLineupPath = new Path (new BezierLine(Shooting,firstLineup));
        firstLineupPath.setConstantHeadingInterpolation(firstLineup.getHeading());

        firstPickupPath = new Path (new BezierLine(firstLineup,firstPickup));
        firstPickupPath.setConstantHeadingInterpolation(firstPickup.getHeading());

        shootFirstPickupPath = new Path (new BezierLine(firstPickup,Shooting));
        shootFirstPickupPath.setConstantHeadingInterpolation(Shooting.getHeading());

        movingOffLinePath = new Path(new BezierLine(Shooting, movingOffLine));
        movingOffLinePath.setConstantHeadingInterpolation(Shooting.getHeading());

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

                    if(!robot.timerInitted[8]) {//very very first thing to happen
                        robot.timeArray[8] = robot.currentTime.milliseconds();
                        robot.timerInitted[8] = true;
                    }

                    if (robot.currentTime.milliseconds() > robot.timeArray[8] + 6100) {//Last thing to happen
                        setPathState(2);
                        robot.timerInitted[8] = false;
                    }

                    else if (robot.currentTime.milliseconds() > robot.timeArray[8] + 6000) {//Last thing to happen

                        robot.shoot();
                    }

                    else if (robot.currentTime.milliseconds() > robot.timeArray[8] + 3000) {

                        robot.shoot();
                    }

                    else {//Second thing to happen
                        robot.R_shooter.setPower(robot.setting_ShooterRPM());
                        robot.L_shooter.setPower(robot.setting_ShooterRPM());
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

                    if(!robot.timerInitted[9]) {//very very first thing to happen
                        robot.timeArray[9] = robot.currentTime.milliseconds();
                        robot.timerInitted[9] = true;
                    }

                    if (robot.currentTime.milliseconds() > robot.timeArray[9] + 5100) {//Last thing to happen
                        setPathState(6);
                        robot.timerInitted[9] = false;
                    }

                    else if (robot.currentTime.milliseconds() > robot.timeArray[9] + 5000) {//Last thing to happen

                        robot.shoot();
                    }

                    else if (robot.currentTime.milliseconds() > robot.timeArray[9] + 2500) {

                        robot.shoot();
                    }

                    else {//Second thing to happen
                        robot.R_shooter.setPower(robot.setting_ShooterRPM());
                        robot.L_shooter.setPower(robot.setting_ShooterRPM());
                        robot.shoot();

                    }

                }
                break;
            case 6:
                if(!follower.isBusy()) {

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

        robot.turret(true, 1, true, 6000);

        robot.R_shooter.setPower(robot.setting_ShooterRPM());
        robot.L_shooter.setPower(robot.setting_ShooterRPM());

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

        robot.initAprilTag(hardwareMap);

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
        setPathState(0);
    }

    @Override
    public void stop() {
    }
}
