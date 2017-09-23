package org.usfirst.frc.team4028.robot.auton.modes;

import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.auton.actions.HangGearAction;
import org.usfirst.frc.team4028.robot.auton.actions.RunMotionProfileAction;
import org.usfirst.frc.team4028.robot.profiles.CenterGearTrajectory;

// Hangs the center gear
public class CenterGear extends AutonBase {
	@Override
	public void routine() {
		runAction(new RunMotionProfileAction(new CenterGearTrajectory()));
		runAction(new HangGearAction());
	}
}