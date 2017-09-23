package org.usfirst.frc.team4028.robot.paths;

import java.util.ArrayList;

import org.usfirst.frc.team4028.util.control.Path;
import org.usfirst.frc.team4028.util.motion.RigidTransform;
import org.usfirst.frc.team4028.util.motion.Rotation;
import org.usfirst.frc.team4028.util.motion.Translation;
import org.usfirst.frc.team4028.robot.paths.PathBuilder.Waypoint;;

public class CenterGearToShootRed implements PathContainer{
	@Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(89, 160, 0, 0));
        sWaypoints.add(new Waypoint(40, 160, 36, 80));
        sWaypoints.add(new Waypoint(40, 90, 0, 80));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public RigidTransform getStartPose() {
        return new RigidTransform(new Translation(90, 160), Rotation.fromDegrees(180.0));
    }

    @Override
    public boolean isReversed() {
        return false;
    }
    // WAYPOINT_DATA:
    // [{"position":{"x":90,"y":160},"speed":0,"radius":0,"comment":""},{"position":{"x":40,"y":160},"speed":60,"radius":40,"comment":""},{"position":{"x":40,"y":220},"speed":60,"radius":0,"comment":""}]
    // IS_REVERSED: false
    // FILE_NAME: CenterGearToShootBlue
}