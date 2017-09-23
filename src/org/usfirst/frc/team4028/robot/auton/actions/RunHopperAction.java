package org.usfirst.frc.team4028.robot.auton.actions;

import org.usfirst.frc.team4028.robot.subsystems.Hopper;
import org.usfirst.frc.team4028.robot.subsystems.Hopper.HOPPER_STATE;

public class RunHopperAction implements Action{
	private Hopper _hopper = Hopper.getInstance();
	
	@Override
	public void start() {
		_hopper.setState(HOPPER_STATE.FORWARD);
	}

	@Override
	public void update() {
	}

	@Override
	public void done() {
		_hopper.setState(HOPPER_STATE.STOP);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}