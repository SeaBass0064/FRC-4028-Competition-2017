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
import org.usfirst.frc.team4028.robot.profiles.CenterGearTrajectory;
import org.usfirst.frc.team4028.robot.profiles.JTurn;

public class CenterGearAndShoot extends AutonBase {
	
	@Override
	public void routine() {
		runAction(new RunMotionProfileAction(new CenterGearTrajectory()));
		runAction(new HangGearAction());
		runAction(new ParallelAction(Arrays.asList(new RunShooterSetDistanceAction(120.0), new RunMotionProfileAction(new JTurn(0, false)))));
		runAction(new ParallelAction(Arrays.asList(new RunShooterWithVisionAction(),
				new SeriesAction(Arrays.asList(new AimAtBoilerAction(), new RunHopperAction())))));
	}
}