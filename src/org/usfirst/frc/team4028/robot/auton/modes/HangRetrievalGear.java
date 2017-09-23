package org.usfirst.frc.team4028.robot.auton.modes;

import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.auton.actions.HangGearAction;
import org.usfirst.frc.team4028.robot.auton.actions.RunMotionProfileAction;
import org.usfirst.frc.team4028.robot.profiles.SideGearTrajectory;

public class HangRetrievalGear extends AutonBase {
	boolean _isBlueAlliance;
	
	public HangRetrievalGear(boolean isBlueAlliance) {
		_isBlueAlliance = isBlueAlliance;
	}
	
	@Override
	public void routine() {
		if (_isBlueAlliance) {
			runAction(new RunMotionProfileAction(new SideGearTrajectory(-1.0, true)));
		} else {
			runAction(new RunMotionProfileAction(new SideGearTrajectory(1.0, false)));
		}
		runAction(new HangGearAction());
	}
}