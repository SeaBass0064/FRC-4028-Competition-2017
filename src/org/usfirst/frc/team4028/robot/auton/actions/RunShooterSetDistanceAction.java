package org.usfirst.frc.team4028.robot.auton.actions;

import org.usfirst.frc.team4028.robot.subsystems.Shooter;

// Runs shooter using values at specified distance
public class RunShooterSetDistanceAction implements Action {
	private Shooter _shooter = Shooter.getInstance();
	
	private double _targetDistanceInInches;
	
	public RunShooterSetDistanceAction(double distanceInInches) {
		_targetDistanceInInches = distanceInInches;
	}
	
	@Override
	public void start() {
		_shooter.setTargetDistanceInInches(_targetDistanceInInches);
		_shooter.runAtTargetSpeed();
	}

	@Override
	public void update() {
		_shooter.runAtTargetSpeed();
	}

	@Override
	public void done() {}

	@Override
	public boolean isFinished() {
		return true;
	}
}