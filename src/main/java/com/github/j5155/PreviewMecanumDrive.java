package com.github.j5155;



import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;

import java.lang.Math;
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
        kinematics = new MecanumKinematics(
                IN_PER_TICK * TRACK_WIDTH_TICKS, LATERAL_MULTIPLIER);
        defaultTurnConstraints = new TurnConstraints(
                MAX_ANG_VEL, -MAX_ANG_ACCEL, MAX_ANG_ACCEL);
        defaultVelConstraint =
        new MinVelConstraint(Arrays.asList(
                kinematics.new WheelVelConstraint(MAX_WHEEL_VEL),
                new AngularVelConstraint(MAX_ANG_VEL)
        ));
        defaultAccelConstraint =
                new ProfileAccelConstraint(MIN_PROFILE_ACCEL, MAX_PROFILE_ACCEL);

    }


    public MecanumKinematics kinematics;

    public TurnConstraints defaultTurnConstraints;
    public VelConstraint defaultVelConstraint;
    public final AccelConstraint defaultAccelConstraint;
    public static final class FollowTrajectoryAction implements Action {
        public final TimeTrajectory timeTrajectory;
        private final double[] xPoints, yPoints;
        private double beginTs = -1;
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
            double t;

            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= timeTrajectory.duration) {
                return false;
            }

            Pose2dDual<Time> txWorldTarget = timeTrajectory.get(t);

            p.put("x", txWorldTarget.position.x);
            p.put("y", txWorldTarget.position.y);
            p.put("heading (deg)", Math.toDegrees(txWorldTarget.heading.real.get(0)));



            // only draw when active; only one drive action should be active at a time
            Canvas c = p.fieldOverlay();

            c.setStroke("#4CAF50");
            drawRobot(c, txWorldTarget.value());



            c.setStroke("#4CAF50FF");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);

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
        private double beginTs = -1;
        public TurnAction(TimeTurn turn) {
            this.turn = turn;
        }

        @Override
        public boolean run(TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= turn.duration) {

                return false;
            }

            Pose2dDual<Time> txWorldTarget = turn.get(t);


            Canvas c = p.fieldOverlay();


            c.setStroke("#4CAF50");
            drawRobot(c, txWorldTarget.value());


            c.setStroke("#7C4DFFFF");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);

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