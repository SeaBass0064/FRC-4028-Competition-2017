package org.usfirst.frc.team4028.robot.subsystems;

import org.usfirst.frc.team4028.robot.Constants;
import org.usfirst.frc.team4028.util.LogData;
import org.usfirst.frc.team4028.util.loops.Loop;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GearHandler extends Subsystem{
	private static GearHandler _instance = new GearHandler();
	public static GearHandler getInstance() {
		return _instance;
	}
	
	private static final int 	TILT_PID_P_PROFILE = 0;
	private static final double TILT_PID_P_CONSTANT = 1.6;
	private static final double TILT_PID_I_CONSTANT = 0.0;
	private static final double TILT_PID_D_CONSTANT = 50.0;
		
	private static final double TILT_MAX_V_DOWN_TILT = +3.0; // Down is positive (RIP MAXIMUM...)
	private static final double TILT_MAX_V_UP_TILT = -6.0;
	
	private CANTalon _gearTiltMotor, _gearInfeedMtr;
	
	private static final double GEAR_TILT_AXIS_HOME_POSITION_IN_ROTATIONS = 00.00;
	private static final double GEAR_TILT_SCORING_POSITION_IN_ROTATIONS = 0.1;
	private static final double GEAR_TILT_CHANGE_TO_V_BUS_POSITION_IN_ROTATIONS = 00.48;
	private static final double TARGET_DEADBAND = 00.03;
	
	private static final double GEAR_MOVE_TO_HOME_VELOCITY_CMD = -0.40;   //set
	
	private static final double SEC_FIRST_CHANGE = 0.35;
	private static final double SEC_SECOND_CHANGE = 0.75;
	private static final double MAX_TIME_BEFORE_ABORT_IN_MSEC = 2.0; 
	
	private static final double GEAR_TILT_SPEED = 0.2;
	private static final double GEAR_OUTFEED_SPEED = -1.0;
	
	private GEAR_TILT_STATE _gearTiltState;
	private boolean _isHomed;
	private double _hangGearSeqStartTime;
	
	public enum GEAR_TILT_STATE {
		HOMING,
		HOLD_SCORING_POSITION,
		HOLD_HOME_POSITION,
		HOLD_FLOOR_POSITION,
		RUNNING_HANG_GEAR,
		FALL_TO_FLOOR
	}
	
	private final Loop _loop = new Loop() {
		static final double HOMING_TIME_SECONDS = 3.0;
		GEAR_TILT_STATE _lastState = GEAR_TILT_STATE.FALL_TO_FLOOR;
		double _homingStartTime = 0.0;
		
		@Override
		public void onStart(double timestamp) {
			synchronized(GearHandler.this) {
				if (!_isHomed) {
					_gearTiltState = GEAR_TILT_STATE.HOMING;
				}
			}
		}
		
		@Override
		public void onLoop(double timestamp) {
			synchronized (GearHandler.this) {
				switch(_gearTiltState) {
					case HOMING:
						if (_gearTiltState != _lastState) {
							setPercentVBusCmd(GEAR_MOVE_TO_HOME_VELOCITY_CMD);
	                        _homingStartTime = Timer.getFPGATimestamp();
	                    } 
						
						if (Timer.getFPGATimestamp() >= _homingStartTime + HOMING_TIME_SECONDS) {
	                        stopHoming(false);
	                        DriverStation.reportError("Gear Tilt Zero Timeout", false);
	                    } else if (!_gearTiltMotor.isRevLimitSwitchClosed()) {
	                    	stopHoming(true);
	                    }
						break;
						
					case HOLD_FLOOR_POSITION:
						setPositionCmd(GEAR_TILT_CHANGE_TO_V_BUS_POSITION_IN_ROTATIONS);
						if (_gearTiltMotor.getPosition() >= (GEAR_TILT_CHANGE_TO_V_BUS_POSITION_IN_ROTATIONS - TARGET_DEADBAND)) {
							_gearTiltState = GEAR_TILT_STATE.FALL_TO_FLOOR;
						}
						break;
						
					case HOLD_HOME_POSITION:
						setPositionCmd(GEAR_TILT_AXIS_HOME_POSITION_IN_ROTATIONS);
						break;
						
					case FALL_TO_FLOOR:
						setPercentVBusCmd(0.0);
						break;
						
					case HOLD_SCORING_POSITION:
						setPositionCmd(GEAR_TILT_SCORING_POSITION_IN_ROTATIONS);
						break;
						
					case RUNNING_HANG_GEAR:
						if (_gearTiltState != _lastState) {
							_hangGearSeqStartTime = Timer.getFPGATimestamp();
						}
						
						break;
				}
				_lastState = _gearTiltState;
			}
		}
		
		@Override
		public void onStop(double timestamp) {
			synchronized(GearHandler.this) {
				if (_gearTiltState == GEAR_TILT_STATE.HOMING) {
					stopHoming(false);
				}
			}
		}
	};
	
	public Loop getLoop() {
		return _loop;
	}
	
	private GearHandler() {
		// Tilt Motor
		_gearTiltMotor = new CANTalon(Constants.GEAR_TILT_CAN_BUS_ADDR);
		_gearTiltMotor.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		_gearTiltMotor.enableBrakeMode(false);
		_gearTiltMotor.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		_gearTiltMotor.reverseSensor(false);
		_gearTiltMotor.enableLimitSwitch(false, true);
		_gearTiltMotor.ConfigRevLimitSwitchNormallyOpen(false);
		
		_gearTiltMotor.setProfile(TILT_PID_P_PROFILE);
		_gearTiltMotor.setPID(TILT_PID_P_CONSTANT, TILT_PID_I_CONSTANT, TILT_PID_D_CONSTANT);
		_gearTiltMotor.configNominalOutputVoltage(0.0f, -0.0f);
		_gearTiltMotor.configPeakOutputVoltage(TILT_MAX_V_DOWN_TILT, TILT_MAX_V_UP_TILT);
		
		// Infeed Motor
		_gearInfeedMtr = new CANTalon(Constants.GEAR_INFEED_CAN_BUS_ADDR);
		_gearInfeedMtr.changeControlMode(TalonControlMode.PercentVbus);
		_gearInfeedMtr.enableBrakeMode(false);
		_gearInfeedMtr.enableLimitSwitch(false, false);
	}
	
	private synchronized void setPositionCmd(double positionCmd) {
		if (_gearTiltMotor.getControlMode() != TalonControlMode.Position) {
			_gearTiltMotor.changeControlMode(TalonControlMode.Position);
		}
		_gearTiltMotor.set(positionCmd);
	}
	
	private synchronized void setPercentVBusCmd(double vBusCmd) {
		if (_gearTiltMotor.getControlMode() != TalonControlMode.PercentVbus) {
			_gearTiltMotor.changeControlMode(TalonControlMode.PercentVbus);
		}
		_gearTiltMotor.set(vBusCmd);
	}
	
	public synchronized void setState(GEAR_TILT_STATE state) {
		_gearTiltState = state;
	}
	
	public synchronized void hangGearReentrant() {
		// safety valve since in this mode we take away operator control temporarily
		double elapsedTimeInMSec = Timer.getFPGATimestamp() - _hangGearSeqStartTime;

		if(elapsedTimeInMSec < SEC_FIRST_CHANGE) {  //Initial State of Gear
			setPercentVBusCmd(GEAR_TILT_SPEED);
			spinInfeedWheels(GEAR_OUTFEED_SPEED);
		}
		else if(elapsedTimeInMSec > SEC_FIRST_CHANGE && elapsedTimeInMSec <= SEC_SECOND_CHANGE) { // second state of gear Sequence
			stop();
		}
		else if(elapsedTimeInMSec > SEC_SECOND_CHANGE && elapsedTimeInMSec < MAX_TIME_BEFORE_ABORT_IN_MSEC) { //final state of gear sequence
			_gearTiltState = GEAR_TILT_STATE.HOLD_SCORING_POSITION;
		}
		else if(elapsedTimeInMSec >= MAX_TIME_BEFORE_ABORT_IN_MSEC) { //timeout in order to end sequence
			DriverStation.reportWarning("=!=!= HangGearInTeleopSequence Timeout ABORT =!=!=", false);
		}
	}
	
	public synchronized boolean isHomed() {
		return _isHomed;
	}
	
	private synchronized void stopHoming(boolean success) {
        if (success) {
            _isHomed = true;
            _gearTiltState = GEAR_TILT_STATE.HOLD_HOME_POSITION;
            zeroSensors();
        } else {
            _gearTiltState = GEAR_TILT_STATE.FALL_TO_FLOOR;
        }
        setPercentVBusCmd(0.0);
    }
	
	public synchronized void spinInfeedWheels(double percentVBusCmd) {
		_gearInfeedMtr.set(-0.5 * percentVBusCmd);
	}
	
	public synchronized GEAR_TILT_STATE getState() {
		return _gearTiltState;
	}
	
	@Override
	public synchronized void stop() {
		setPercentVBusCmd(0.0);
		spinInfeedWheels(0.0);
	}
	
	@Override
	public synchronized void zeroSensors() {
		_gearTiltMotor.setPosition(0.0);
	}
	
	@Override
	public void outputToSmartDashboard() {
		String gearInFeedMtrData = "?";
		if(Math.abs(_gearInfeedMtr.getOutputVoltage()) > 0) {
			gearInFeedMtrData = String.format("%s (%.0f%%)", 
												"ON", 
												(_gearInfeedMtr.getOutputVoltage() / _gearInfeedMtr.getBusVoltage())* 100);
		} else {
			gearInFeedMtrData = String.format("%s (%.0f%%)", "off", 0.0);
		}
		
		SmartDashboard.putString("Gear In/OutFeed Cmd", gearInFeedMtrData);	
	}
	
	@Override
	public void updateLogData(LogData logData) {
		logData.AddData("Gear:TiltPos", String.format("%.2f", _gearTiltMotor.getPosition()));
		logData.AddData("Gear:Tilt%VBus", String.format("%.4f", (_gearTiltMotor.getOutputVoltage()) / _gearTiltMotor.getBusVoltage()));
		logData.AddData("Gear:Outfeed%Vbus", String.format("%.2f", _gearTiltMotor.get()));
	}
}