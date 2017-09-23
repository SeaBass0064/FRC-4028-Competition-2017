package org.usfirst.frc.team4028.robot.auton.actions;

import org.usfirst.frc.team4028.robot.subsystems.Chassis;
import org.usfirst.frc.team4028.robot.subsystems.GearHandler;
import org.usfirst.frc.team4028.robot.subsystems.GearHandler.GEAR_TILT_STATE;

/* Runs the hang gear sequence */
public class HangGearAction implements Action{
	private Chassis _chassis = Chassis.getInstance();
	private GearHandler _gearHandler = GearHandler.getInstance();
	
	public HangGearAction() {}
	
	@Override
	public void start() {
		_gearHandler.setState(GEAR_TILT_STATE.RUNNING_HANG_GEAR);
	}

	@Override
	public void update() {
		_chassis.arcadeDrive(-0.6, 0.0);
	}

	@Override
	public void done() {
		_chassis.stop();
	}

	@Override
	public boolean isFinished() {
		if (_gearHandler.getState() == GEAR_TILT_STATE.HOLD_SCORING_POSITION) {	// Finished when gear handler returns to scoring position
			return true;
		} else {
			return false;
		}
	}
}