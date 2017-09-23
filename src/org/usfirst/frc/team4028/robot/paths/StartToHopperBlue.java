package org.usfirst.frc.team4028.robot.paths;

import java.util.ArrayList;

import org.usfirst.frc.team4028.util.control.Path;
import org.usfirst.frc.team4028.util.motion.RigidTransform;
import org.usfirst.frc.team4028.util.motion.Rotation;
import org.usfirst.frc.team4028.util.motion.Translation;
import org.usfirst.frc.team4028.robot.paths.PathBuilder.Waypoint;

public class StartToHopperBlue implements PathContainer{
	@Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(16, 234, 0, 0));
        sWaypoints.add(new Waypoint(97, 234, 54, 90));
        sWaypoints.add(new Waypoint(97, 295, 0, 90, "RamWall"));
        sWaypoints.add(new Waypoint(97, 300, 0, 40));
        sWaypoints.add(new Waypoint(97, 340, 0, 40));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public RigidTransform getStartPose() {
        return new RigidTransform(new Translation(16, 234), Rotation.fromDegrees(0.0));
    }

    @Override
    public boolean isReversed() {
        return false;
    }
    // WAYPOINT_DATA:
    // [{"position":{"x":16,"y":234},"speed":0,"radius":0,"comment":""},{"position":{"x":95,"y":234},"speed":60,"radius":48,"comment":""},{"position":{"x":95,"y":300},"speed":60,"radius":0,"comment":""}]
    // IS_REVERSED: false
    // FILE_NAME: StartToHopperBlue
}