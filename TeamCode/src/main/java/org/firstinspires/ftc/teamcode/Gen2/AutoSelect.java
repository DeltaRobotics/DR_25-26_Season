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
                    l.add("9", AutoBlue9Close::new);
                });
                z.folder("Far", a -> {
                    a.add("9", AutoBlue9Far::new);
                });
            });
            s.folder("Red", z -> {
                z.folder("Close", l -> {
                    l.add("12", AutoRed12Close::new);
                    l.add("9", AutoRed9Close::new);
                });
                z.folder("Far", a -> {
                    a.add("9", AutoRed9Far::new);
                });
            });

        });
    }


}



