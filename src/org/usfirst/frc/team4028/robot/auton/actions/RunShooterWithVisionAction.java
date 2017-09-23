package org.usfirst.frc.team4028.robot.auton.actions;

import org.usfirst.frc.team4028.robot.sensors.RoboRealmClient;
import org.usfirst.frc.team4028.robot.subsystems.Shooter;

public class RunShooterWithVisionAction implements Action{
	private Shooter _shooter = Shooter.getInstance();
	private RoboRealmClient _roboRealm = RoboRealmClient.getInstance();
	public RunShooterWithVisionAction() {
	}
	
	@Override
	public void start() {
		_shooter.setTargetDistanceInInches(_roboRealm.get_DistanceToBoilerInches());
		_shooter.runAtTargetSpeed();
	}

	@Override
	public void update() {
		_shooter.setTargetDistanceInInches(_roboRealm.get_DistanceToBoilerInches());
		_shooter.runAtTargetSpeed();
	}

	@Override
	public void done() {
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}