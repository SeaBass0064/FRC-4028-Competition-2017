package org.usfirst.frc.team4028.robot.paths;

import org.usfirst.frc.team4028.robot.paths.profiles.PathAdapter;
import org.usfirst.frc.team4028.util.control.Path;
import org.usfirst.frc.team4028.util.motion.RigidTransform;

public class StartToBoilerGearBlue implements PathContainer{
	@Override
    public Path buildPath() {
        return PathAdapter.getBlueGearPath();
    }

    @Override
    public RigidTransform getStartPose() {
        return PathAdapter.getBlueStartPose();
    }

    @Override
    public boolean isReversed() {
        return true;
    }
}