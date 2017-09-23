package org.usfirst.frc.team4028.robot.paths;

import org.usfirst.frc.team4028.robot.paths.profiles.PathAdapter;
import org.usfirst.frc.team4028.util.control.Path;
import org.usfirst.frc.team4028.util.motion.RigidTransform;

public class StartToBoilerGearRed implements PathContainer{
	 @Override
    public Path buildPath() {
        return PathAdapter.getRedGearPath();
    }

    @Override
    public RigidTransform getStartPose() {
        return PathAdapter.getRedStartPose();
    }

    @Override
    public boolean isReversed() {
        return true;
    }
}