package org.usfirst.frc.team4028.robot.paths;

import java.util.List;

import org.usfirst.frc.team4028.util.control.Path;
import org.usfirst.frc.team4028.util.control.PathSegment;
import org.usfirst.frc.team4028.util.motion.RigidTransform;
import org.usfirst.frc.team4028.util.motion.Rotation;
import org.usfirst.frc.team4028.util.motion.Translation;

public class PathBuilder {
	private static final double kEpsilon = 1E-9;
    private static final double kReallyBigNumber = 1E9;

    public static Path buildPathFromWaypoints(List<Waypoint> w) {
        Path p = new Path();
        if (w.size() < 2)
            throw new Error("Path must contain at least 2 waypoints");
        int i = 0;
        if (w.size() > 2) {
            do {
                new Arc(getPoint(w, i), getPoint(w, i + 1), getPoint(w, i + 2)).addToPath(p);
                i++;
            } while (i < w.size() - 2);
        }
        new Line(w.get(w.size() - 2), w.get(w.size() - 1)).addToPath(p, 0);
        p.extrapolateLast();
        p.verifySpeeds();
        // System.out.println(p);
        return p;
    }

    private static Waypoint getPoint(List<Waypoint> w, int i) {
        if (i > w.size())
            return w.get(w.size() - 1);
        return w.get(i);
    }

    /**
     * A waypoint along a path. Contains a position, radius (for creating curved paths), and speed. The information from
     * these waypoints is used by the PathBuilder class to generate Paths. Waypoints also contain an optional marker
     * that is used by the WaitForPathMarkerAction.
     *
     * @see PathBuilder
     * @see WaitForPathMarkerAction
     */
    public static class Waypoint {
        Translation position;
        double radius;
        double speed;
        String marker;

        public Waypoint(Waypoint other) {
            this(other.position.x(), other.position.y(), other.radius, other.speed, other.marker);
        }

        public Waypoint(double x, double y, double r, double s) {
            position = new Translation(x, y);
            radius = r;
            speed = s;
        }

        public Waypoint(Translation pos, double r, double s) {
            position = pos;
            radius = r;
            speed = s;
        }

        public Waypoint(double x, double y, double r, double s, String m) {
            position = new Translation(x, y);
            radius = r;
            speed = s;
            marker = m;
        }
    }

    /**
     * A Line object is formed by two Waypoints. Contains a start and end position, slope, and speed.
     */
    static class Line {
        Waypoint a;
        Waypoint b;
        Translation start;
        Translation end;
        Translation slope;
        double speed;

        public Line(Waypoint a, Waypoint b) {
            this.a = a;
            this.b = b;
            slope = new Translation(a.position, b.position);
            speed = b.speed;
            start = a.position.translateBy(slope.scale(a.radius / slope.norm()));
            end = b.position.translateBy(slope.scale(-b.radius / slope.norm()));
        }

        private void addToPath(Path p, double endSpeed) {
            double pathLength = new Translation(end, start).norm();
            if (pathLength > kEpsilon) {
                if (b.marker != null) {
                    p.addSegment(new PathSegment(start.x(), start.y(), end.x(), end.y(), b.speed,
                            p.getLastMotionState(), endSpeed, b.marker));
                } else {
                    p.addSegment(new PathSegment(start.x(), start.y(), end.x(), end.y(), b.speed,
                            p.getLastMotionState(), endSpeed));
                }
            }

        }
    }

    /**
     * An Arc object is formed by two Lines that share a common Waypoint. Contains a center position, radius, and speed.
     */
    static class Arc {
        Line a;
        Line b;
        Translation center;
        double radius;
        double speed;

        public Arc(Waypoint a, Waypoint b, Waypoint c) {
            this(new Line(a, b), new Line(b, c));
        }

        public Arc(Line a, Line b) {
            this.a = a;
            this.b = b;
            this.speed = (a.speed + b.speed) / 2;
            this.center = intersect(a, b);
            this.radius = new Translation(center, a.end).norm();
        }

        private void addToPath(Path p) {
            a.addToPath(p, speed);
            if (radius > kEpsilon && radius < kReallyBigNumber) {
                p.addSegment(new PathSegment(a.end.x(), a.end.y(), b.start.x(), b.start.y(), center.x(), center.y(),
                        speed, p.getLastMotionState(), b.speed));
            }
        }

        private static Translation intersect(Line l1, Line l2) {
            final RigidTransform lineA = new RigidTransform(l1.end, new Rotation(l1.slope, true).normal());
            final RigidTransform lineB = new RigidTransform(l2.start, new Rotation(l2.slope, true).normal());
            return lineA.intersection(lineB);
        }
    }
}