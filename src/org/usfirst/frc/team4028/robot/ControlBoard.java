package org.usfirst.frc.team4028.robot;

import org.usfirst.frc.team4028.util.LatchedBoolean;
import org.usfirst.frc.team4028.util.LogitechF310;

import edu.wpi.first.wpilibj.Joystick;

public class ControlBoard {
	private static ControlBoard _instance = new ControlBoard();
	public static ControlBoard getInstance() {
		return _instance;
	}
	
	private final Joystick _driverGamepad;
	private final Joystick _operatorGamepad;
	
	// Latched Boolean Buttons (to check when a button is just pressed)
	private LatchedBoolean _shiftGearBtn = new LatchedBoolean();
	private LatchedBoolean _toggleShooterBtn = new LatchedBoolean();
	private LatchedBoolean _moveSliderDownBtn = new LatchedBoolean();
	private LatchedBoolean _moveSliderUpBtn = new LatchedBoolean();
	private LatchedBoolean _swapCameraBtn = new LatchedBoolean();
	
	private ControlBoard() {
		_driverGamepad = new Joystick(0);
		_operatorGamepad = new Joystick(1);
	}
	
	/* Axis Inputs */
	// Driver
	public double getThrottleCmd() {
		return _driverGamepad.getRawAxis(LogitechF310.LEFT_Y_AXIS);
	}
	
	public double getTurnCmd() {
		return _driverGamepad.getRawAxis(LogitechF310.RIGHT_X_AXIS);
	}
	
	public double getSpinChassisLeftCmd() {
		return _driverGamepad.getRawAxis(LogitechF310.LEFT_TRIGGER);
	}
	
	public double getSpinChassisRightCmd() {
		return _driverGamepad.getRawAxis(LogitechF310.RIGHT_TRIGGER);
	}
	
	// Operator
	public double getFuelInfeedOutfeedJoystickCmd() {
		return _operatorGamepad.getRawAxis(LogitechF310.LEFT_Y_AXIS);
	}
	
	public double getClimberSpeedCmd() {
		return _operatorGamepad.getRawAxis(LogitechF310.RIGHT_Y_AXIS);
	}
	
	public boolean getIsVisionAimPressed() {
		return _operatorGamepad.getRawAxis(LogitechF310.LEFT_TRIGGER) > 0.1;
	}
	
	public boolean getIsFireBallPressed() {
		return _operatorGamepad.getRawAxis(LogitechF310.RIGHT_TRIGGER) > 0.1;
	}
	
	/* Button Inputs */
	// Driver
	public boolean getIsSendGearTiltToFloorPressed() {
		return _driverGamepad.getRawButton(LogitechF310.GREEN_BUTTON_A);
	}
	
	public boolean getIsSendGearTiltToScorePressed() {
		return _driverGamepad.getRawButton(LogitechF310.RED_BUTTON_B);
	}
	
	public boolean getIsStartGearScoreSequencePressed() {
		return _driverGamepad.getRawButton(LogitechF310.BLUE_BUTTON_X);
	}
	
	public boolean getIsSendGearTiltToHomePressed() {
		return _driverGamepad.getRawButton(LogitechF310.YELLOW_BUTTON_Y);
	}
	
	public boolean getIsRezeroGearTiltPressed() {
		return _driverGamepad.getRawButton(LogitechF310.BACK_BUTTON);
	}
	
	public boolean getIsShiftGearJustPressed() {
		return _shiftGearBtn.isJustPressed(_driverGamepad.getRawButton(LogitechF310.START_BUTTON));
	}
	
	public boolean getIsInfeedGearPressed() {
		return _driverGamepad.getRawButton(LogitechF310.LEFT_BUMPER);
	}
	
	public boolean getIsOutfeedGearPressed() {
		return _driverGamepad.getRawButton(LogitechF310.RIGHT_BUMPER);
	}
	
	// Operator
	public boolean getIsToggleShooterMtrsJustPressed() {
		return _toggleShooterBtn.isJustPressed(_operatorGamepad.getRawButton(LogitechF310.GREEN_BUTTON_A));
	}
	
	public boolean getIsMoveShooterSlideDownJustPressed() {
		return _moveSliderDownBtn.isJustPressed(_operatorGamepad.getRawButton(LogitechF310.RED_BUTTON_B));
	}
	
	public boolean getIsSwapCameraJustPressed() {
		return _swapCameraBtn.isJustPressed(_operatorGamepad.getRawButton(LogitechF310.BLUE_BUTTON_X));
	}
	
	public boolean getIsMoveShooterSliderUpPressed() {
		return _moveSliderUpBtn.isJustPressed(_operatorGamepad.getRawButton(LogitechF310.YELLOW_BUTTON_Y));
	}
	
	public boolean getIsRunFeederInReversePressed() {
		return _operatorGamepad.getRawButton(LogitechF310.START_BUTTON);
	}
	
	public boolean getIsAutoAimDistancePressed() {
		return _operatorGamepad.getRawButton(LogitechF310.BACK_BUTTON);
	}
}