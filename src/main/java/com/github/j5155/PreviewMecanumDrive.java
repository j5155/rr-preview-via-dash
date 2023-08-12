package com.github.j5155;



import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;

import java.util.Arrays;
import java.util.List;

public final class PreviewMecanumDrive {
    // drive model parameters
    public double IN_PER_TICK;
    public double LATERAL_IN_PER_TICK;
    public double TRACK_WIDTH_TICKS;
    public double LATERAL_MULTIPLIER;

    // path profile parameters
    public double MAX_WHEEL_VEL;
    public double MIN_PROFILE_ACCEL;
    public double MAX_PROFILE_ACCEL;

    // turn profile parameters
    public double MAX_ANG_VEL;
    public double MAX_ANG_ACCEL;

    public PreviewMecanumDrive(double IN_PER_TICK, double LATERAL_IN_PER_TICK, double TRACK_WIDTH_TICKS, double LATERAL_MULTIPLIER,
                               double MAX_WHEEL_VEL, double MIN_PROFILE_ACCEL, double MAX_PROFILE_ACCEL, double MAX_ANG_VEL, double MAX_ANG_ACCEL) {
        this.IN_PER_TICK = IN_PER_TICK;
        this.LATERAL_IN_PER_TICK = LATERAL_IN_PER_TICK;
        this.TRACK_WIDTH_TICKS = TRACK_WIDTH_TICKS;
        this.LATERAL_MULTIPLIER = LATERAL_MULTIPLIER;
        this.MAX_WHEEL_VEL = MAX_WHEEL_VEL;
        this.MIN_PROFILE_ACCEL = MIN_PROFILE_ACCEL;
        this.MAX_PROFILE_ACCEL = MAX_PROFILE_ACCEL;
        this.MAX_ANG_VEL = MAX_ANG_VEL;
        this.MAX_ANG_ACCEL = MAX_ANG_ACCEL;
    }


    public final MecanumKinematics kinematics = new MecanumKinematics(
            IN_PER_TICK * TRACK_WIDTH_TICKS, LATERAL_MULTIPLIER);

    public final TurnConstraints defaultTurnConstraints = new TurnConstraints(
            MAX_ANG_VEL, -MAX_ANG_ACCEL, MAX_ANG_ACCEL);
    public final VelConstraint defaultVelConstraint =
            new MinVelConstraint(Arrays.asList(
                    kinematics.new WheelVelConstraint(MAX_WHEEL_VEL),
                    new AngularVelConstraint(MAX_ANG_VEL)
            ));
    public final AccelConstraint defaultAccelConstraint =
            new ProfileAccelConstraint(MIN_PROFILE_ACCEL, MAX_PROFILE_ACCEL);
    public static final class FollowTrajectoryAction implements Action {
        public final TimeTrajectory timeTrajectory;
        private final double[] xPoints, yPoints;
        public FollowTrajectoryAction(TimeTrajectory t) {
            timeTrajectory = t;

            List<Double> disps = com.acmerobotics.roadrunner.Math.range(
                    0, t.path.length(),
                    (int) Math.ceil(t.path.length() / 2));
            xPoints = new double[disps.size()];
            yPoints = new double[disps.size()];
            for (int i = 0; i < disps.size(); i++) {
                Pose2d p = t.path.get(disps.get(i), 1).value();
                xPoints[i] = p.position.x;
                yPoints[i] = p.position.y;
            }
        }
        @Override
        public boolean run(TelemetryPacket p) {
            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#4CAF507A");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);
        }
    }
    public static final class TurnAction implements Action {
        private final TimeTurn turn;
        public TurnAction(TimeTurn turn) {
            this.turn = turn;
        }

        @Override
        public boolean run(TelemetryPacket p) {
            return true;
        }
        @Override
        public void preview(Canvas c) {
            c.setStroke("#7C4DFF7A");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
        }
    }
    public static void drawRobot(Canvas c, Pose2d t) {
        final double ROBOT_RADIUS = 9;

        c.setStrokeWidth(1);
        c.strokeCircle(t.position.x, t.position.y, ROBOT_RADIUS);

        Vector2d halfv = t.heading.vec().times(0.5 * ROBOT_RADIUS);
        Vector2d p1 = t.position.plus(halfv);
        Vector2d p2 = p1.plus(halfv);
        c.strokeLine(p1.x, p1.y, p2.x, p2.y);
    }
    public TrajectoryActionBuilder actionBuilder(Pose2d beginPose) {
        return new TrajectoryActionBuilder(
                TurnAction::new,
                FollowTrajectoryAction::new,
                beginPose, 1e-6, 0.0,
                defaultTurnConstraints,
                defaultVelConstraint, defaultAccelConstraint,
                0.25, 0.1
        );
    }
}