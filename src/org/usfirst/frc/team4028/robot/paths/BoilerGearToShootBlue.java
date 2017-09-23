package org.usfirst.frc.team4028.robot.paths;

import org.usfirst.frc.team4028.util.control.Path;
import org.usfirst.frc.team4028.util.motion.RigidTransform;
import org.usfirst.frc.team4028.util.motion.Rotation;
import org.usfirst.frc.team4028.util.motion.Translation;
import org.usfirst.frc.team4028.robot.paths.PathBuilder.Waypoint;

import java.util.ArrayList;

public class BoilerGearToShootBlue implements PathContainer{
	@Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(116, 209, 0, 0));
        sWaypoints.add(new Waypoint(101, 235, 0, 60));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public RigidTransform getStartPose() {
        return new RigidTransform(new Translation(120, 215), Rotation.fromDegrees(120.0));
    }

    @Override
    public boolean isReversed() {
        return false;
    }
    // WAYPOINT_DATA:
    // [{"position":{"x":120,"y":215},"speed":0,"radius":0,"comment":""},{"position":{"x":90,"y":245},"speed":120,"radius":0,"comment":""}]
    // IS_REVERSED: true
    // FILE_NAME: GearToShootBlue
}