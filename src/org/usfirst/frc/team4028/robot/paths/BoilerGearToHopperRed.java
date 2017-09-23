package org.usfirst.frc.team4028.robot.paths;

import org.usfirst.frc.team4028.robot.paths.profiles.PathAdapter;
import org.usfirst.frc.team4028.util.control.Path;
import org.usfirst.frc.team4028.util.motion.RigidTransform;
import org.usfirst.frc.team4028.util.motion.Rotation;
import org.usfirst.frc.team4028.util.motion.Translation;

public class BoilerGearToHopperRed implements PathContainer{
	@Override
    public Path buildPath() {
        return PathAdapter.getRedHopperPath();
    }

    @Override
    public RigidTransform getStartPose() {
        return new RigidTransform(new Translation(112, 115), Rotation.fromDegrees(0.0));
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}