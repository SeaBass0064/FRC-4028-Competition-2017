package org.usfirst.frc.team4028.robot.paths;

import java.util.ArrayList;

import org.usfirst.frc.team4028.util.control.Path;
import org.usfirst.frc.team4028.util.motion.RigidTransform;
import org.usfirst.frc.team4028.util.motion.Rotation;
import org.usfirst.frc.team4028.util.motion.Translation;
import org.usfirst.frc.team4028.robot.paths.PathBuilder.Waypoint;

public class StartToCenterGearRed implements PathContainer{
	@Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        ;
        sWaypoints.add(new Waypoint(16, 160, 0, 0));
        sWaypoints.add(new Waypoint(89, 160, 0, 40));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public RigidTransform getStartPose() {
        return new RigidTransform(new Translation(16, 160), Rotation.fromDegrees(180.0));
    }

    @Override
    public boolean isReversed() {
        return true;
    }
    // WAYPOINT_DATA:
    // [{"position":{"x":16,"y":89},"speed":0,"radius":0,"comment":""},{"position":{"x":80,"y":89},"speed":30,"radius":0,"comment":""},{"position":{"x":109,"y":121},"speed":30,"radius":0,"comment":""}]
    // IS_REVERSED: true
    // FILE_NAME: StartToGearRed
}