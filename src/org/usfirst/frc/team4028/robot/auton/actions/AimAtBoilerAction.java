package org.usfirst.frc.team4028.robot.auton.actions;

import org.usfirst.frc.team4028.robot.subsystems.Chassis;

/* Aims the robot towards a vision target */
public class AimAtBoilerAction implements Action {
	private Chassis _chassis = Chassis.getInstance();
	
	@Override
	public void start() {
		_chassis.aimAtVisionTarget();
	}

	@Override
	public void update() {}

	@Override
	public void done() {
		_chassis.stop();
	}

	@Override
	public boolean isFinished() {
		return Math.abs(_chassis.autoAimError()) < 0.1;		// Action complete when robot within vision deadband
	}
}