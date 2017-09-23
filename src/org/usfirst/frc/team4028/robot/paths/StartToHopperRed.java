package org.usfirst.frc.team4028.robot.paths;

import java.util.ArrayList;

import org.usfirst.frc.team4028.util.control.Path;
import org.usfirst.frc.team4028.util.motion.RigidTransform;
import org.usfirst.frc.team4028.util.motion.Rotation;
import org.usfirst.frc.team4028.util.motion.Translation;
import org.usfirst.frc.team4028.robot.paths.PathBuilder.Waypoint;

public class StartToHopperRed implements PathContainer{
	@Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(16, 90, 0, 0));
        sWaypoints.add(new Waypoint(97, 90, 54, 90));
        sWaypoints.add(new Waypoint(97, 29, 0, 90, "RamWall"));
        sWaypoints.add(new Waypoint(97, 24, 0, 40));
        sWaypoints.add(new Waypoint(97, 4, 0, 40));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public RigidTransform getStartPose() {
        return new RigidTransform(new Translation(16, 90), Rotation.fromDegrees(0.0));
    }

    @Override
    public boolean isReversed() {
        return false;
    }
    // WAYPOINT_DATA:
    // [{"position":{"x":16,"y":90},"speed":0,"radius":0,"comment":""},{"position":{"x":92,"y":90},"speed":90,"radius":54,"comment":""},{"position":{"x":92,"y":29},"speed":90,"radius":0,"comment":""},{"position":{"x":92,"y":24},"speed":40,"radius":0,"comment":""},{"position":{"x":92,"y":4},"speed":40,"radius":0,"comment":""}]
    // IS_REVERSED: false
    // FILE_NAME: StartToHopperRed
}