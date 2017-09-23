package org.usfirst.frc.team4028.robot.auton.actions;

import org.usfirst.frc.team4028.robot.paths.PathContainer;
import org.usfirst.frc.team4028.robot.subsystems.Chassis;
import org.usfirst.frc.team4028.util.control.Path;

/* Runs a motion profile */
public class RunMotionProfileAction implements Action {
	private Chassis _chassis = Chassis.getInstance();
	private PathContainer _pathContainer;
	private Path _path;
	
	public RunMotionProfileAction(PathContainer p) {
		_pathContainer = p;
		_path = _pathContainer.buildPath();
	}
	
	@Override
	public void start() {
		_chassis.setWantDrivePath(_path, _pathContainer.isReversed());
	}

	@Override
	public void update() {}	// Nothing here since trajController updates in its own thread

	@Override
	public void done() {	
		_chassis.stop();
	}

	@Override
	public boolean isFinished() {
		return _chassis.isDoneWithPath();
	}
}