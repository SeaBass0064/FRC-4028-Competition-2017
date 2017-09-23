package org.usfirst.frc.team4028.robot.auton.modes;

import java.util.Arrays;

import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.auton.actions.AimAtBoilerAction;
import org.usfirst.frc.team4028.robot.auton.actions.HangGearAction;
import org.usfirst.frc.team4028.robot.auton.actions.ParallelAction;
import org.usfirst.frc.team4028.robot.auton.actions.RunHopperAction;
import org.usfirst.frc.team4028.robot.auton.actions.RunMotionProfileAction;
import org.usfirst.frc.team4028.robot.auton.actions.RunShooterSetDistanceAction;
import org.usfirst.frc.team4028.robot.auton.actions.RunShooterWithVisionAction;
import org.usfirst.frc.team4028.robot.auton.actions.SeriesAction;
import org.usfirst.frc.team4028.robot.profiles.SideGearTrajectory;

public class HangBoilerGearAndShoot extends AutonBase {
	boolean _isBlueAlliance;
	
	public HangBoilerGearAndShoot(boolean isBlueAlliance) {
		_isBlueAlliance = isBlueAlliance;
	}
	
	@Override
	public void routine() {
		if (_isBlueAlliance) {
			runAction(new RunMotionProfileAction(new SideGearTrajectory(1.0, false)));
		} else {
			runAction(new RunMotionProfileAction(new SideGearTrajectory(-1.0, true)));
		}
		runAction(new ParallelAction(Arrays.asList(new HangGearAction(), new RunShooterSetDistanceAction(120.0))));
		runAction(new ParallelAction(Arrays.asList(new RunShooterWithVisionAction(),
				new SeriesAction(Arrays.asList(new AimAtBoilerAction(), new RunHopperAction())))));
	}
}