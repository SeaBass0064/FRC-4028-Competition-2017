package org.usfirst.frc.team4028.robot.paths;

import org.usfirst.frc.team4028.robot.paths.profiles.PathAdapter;
import org.usfirst.frc.team4028.util.control.Path;
import org.usfirst.frc.team4028.util.motion.RigidTransform;
import org.usfirst.frc.team4028.util.motion.Rotation;
import org.usfirst.frc.team4028.util.motion.Translation;

public class BoilerGearToHopperBlue implements PathContainer{
	@Override
    public Path buildPath() {
        return PathAdapter.getBlueHopperPath();
    }

    @Override
    public RigidTransform getStartPose() {
        return new RigidTransform(new Translation(116, 209), Rotation.fromDegrees(0.0));
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}