package org.usfirst.frc.team4028.robot.auton.actions;

import org.usfirst.frc.team4028.robot.subsystems.Chassis;

// Turns the chassis to a specified angle
public class TurnAction implements Action{
	private Chassis _chassis = Chassis.getInstance();
	
	private double _targetAngle;
	
	public TurnAction(double angle) {
		_targetAngle = angle;
	}
	
	@Override
	public void start() {
		_chassis.setTargetAngle(_targetAngle);
	}

	@Override
	public void update() {
	}

	@Override
	public void done() {	
		_chassis.stop();
	}

	@Override
	public boolean isFinished() {
		return Math.abs(_chassis.autoAimError()) < 1.0;		// Returns true when chassis is within angle deadband
	}
}