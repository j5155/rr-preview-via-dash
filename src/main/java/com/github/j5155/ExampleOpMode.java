package com.github.j5155;



import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.github.j5155.PreviewMecanumDrive;
import com.github.j5155.TestDashboardInstance;

public class ExampleOpMode {
    // TODO: Copy these from your existing MecanumDrive!

    // drive model parameters
    public static double IN_PER_TICK = 0.000549539;
    public static double LATERAL_IN_PER_TICK = 1;
    public static double TRACK_WIDTH_TICKS = 48852.1340223;
    public static double LATERAL_MULTIPLIER = IN_PER_TICK / LATERAL_IN_PER_TICK;

    // path profile parameters
    public static double MAX_WHEEL_VEL = 50;
    public static double MIN_PROFILE_ACCEL = -30;
    public static double MAX_PROFILE_ACCEL = 50;

    // turn profile parameters
    public static double MAX_ANG_VEL = Math.PI; // shared with path
    public static double MAX_ANG_ACCEL = Math.PI;

    public static void main(String[] args) {
        TestDashboardInstance dash = TestDashboardInstance.getInstance();
        dash.start();

        Canvas c = new Canvas();
        PreviewMecanumDrive drive = new PreviewMecanumDrive(IN_PER_TICK, LATERAL_IN_PER_TICK, TRACK_WIDTH_TICKS, LATERAL_MULTIPLIER,
                MAX_WHEEL_VEL, MIN_PROFILE_ACCEL, MAX_PROFILE_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL);

        Pose2d startPose = new Pose2d(0,0,0); // TODO: Update this to reflect your autonomous start position
        TrajectoryActionBuilder trajBuild =
                drive.actionBuilder(startPose) // TODO: Copy your autonomous here
                        .splineTo(new Vector2d(0, 30), Math.PI / 2)
                        .splineTo(new Vector2d(0, 60), Math.PI)
                        .strafeTo(new Vector2d(48,48));
                        // Make sure to remove the .build(), it is not needed here

        Action traj = trajBuild.build();
        traj.preview(c);
        PreviewMecanumDrive.drawRobot(c, startPose);

        while(true) {
            TelemetryPacket p = new TelemetryPacket();
            if (!traj.run(p)) traj = trajBuild.build();
            p.fieldOverlay().getOperations().addAll(c.getOperations());
            dash.core.sendTelemetryPacket(p);
        }


    }
}
