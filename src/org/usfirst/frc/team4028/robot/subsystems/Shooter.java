package org.usfirst.frc.team4028.robot.subsystems;

import java.util.Date;

import org.usfirst.frc.team4028.robot.Constants;
import org.usfirst.frc.team4028.util.GeneralUtilities;
import org.usfirst.frc.team4028.util.LogData;
import org.usfirst.frc.team4028.util.ShooterTable;
import org.usfirst.frc.team4028.util.ShooterTableEntry;
import org.usfirst.frc.team4028.util.loops.Loop;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.VelocityMeasurementPeriod;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends Subsystem {
	private static Shooter _instance = new Shooter();
	public static Shooter getInstance() {
		return _instance;
	}
	
	private CANTalon _firstStgMtr, _secondStgMtr;
	private Servo _linearActuator;
	
	private ShooterTable _shooterTable;
	private ShooterTableEntry _currentShooterTableEntry;
	private ShooterTableEntry _lastShooterTableEntry;
	
	private double _stg1MtrTargetRPM;
	private double _stg2MtrTargetRPM;
	private boolean _isStg1MtrTargetRPMBumpingUp;
	private boolean _isStg2MtrTargetRPMBumpingUp;
	
	private double _currentSliderPosition;
	private boolean _isShooterMotorsReentrantRunning = false;
	
	private long _lastDebugWriteTimeMSec;
	
	// Post Worlds
	/*
	private static final double FIRST_STAGE_MTG_FF_GAIN = 0.027; //0.033; //
	private static final double FIRST_STAGE_MTG_P_GAIN = 0.5; //0.45; // 0.4; //0.35; //0.30; // 0.25;// 0.325;    
	private static final double FIRST_STAGE_MTG_I_GAIN = 0.0;
	private static final double FIRST_STAGE_MTG_D_GAIN = 1.5; //5.0; 	//2.0; //4.0; //7.5; //5.0; //

	private static final double SECOND_STAGE_MTG_FF_GAIN = 0.028; //0.03; //
	private static final double SECOND_STAGE_MTG_P_GAIN = 0.5; // 0.175; 	//0.3; // .250; //0.175; //
	private static final double SECOND_STAGE_MTG_I_GAIN = 0.0;
	private static final double SECOND_STAGE_MTG_D_GAIN = 3.0; //6.0; 	//3.0; //6.0; // 
	*/
	
	// Cleveland	
	private static final double FIRST_STAGE_MTG_FF_GAIN = 0.033; //
	private static final double FIRST_STAGE_MTG_P_GAIN = 0.325;         // 0.3125; // 0.275; //0.225; //0.325; //
	private static final double FIRST_STAGE_MTG_I_GAIN = 0.0;
	private static final double FIRST_STAGE_MTG_D_GAIN = 5.0; 	//2.0; //4.0; //7.5; //5.0; //

	private static final double SECOND_STAGE_MTG_FF_GAIN = 0.03; //
	private static final double SECOND_STAGE_MTG_P_GAIN =  0.175; 	//0.3; // .250; //0.175; //
	private static final double SECOND_STAGE_MTG_I_GAIN = 0.0;
	private static final double SECOND_STAGE_MTG_D_GAIN = 6.0; 	//3.0; //6.0; // 
	
	//==============================================================================================
	
	//define class level Actuator Constants
	private static final double MAX_THRESHOLD_ACTUATOR = 0.80; //0.7; 
	private static final double MIN_THRESHOLD_ACTUATOR = 0.35; //0.4;
	private static final double CHANGE_INTERVAL_ACTUATOR = 0.01;
	private static final double INITIAL_POSITION_ACTUATOR = 0.65;
	
	//define class level Shooter Motor Constants
	private static final double MAX_SHOOTER_RPM = -4400;
	private static final double MIN_SHOOTER_RPM = -2500;
	private static final double SHOOTER_BUMP_RPM = 25;
	private static final double FIRST_STAGE_MTR_DEFAULT_RPM = -3500;
	private static final double SECOND_STAGE_MTR_DEFAULT_RPM = -3200;
	
	private final Loop _loop = new Loop() {
		@Override
		public void onStart(double timestamp) {
			synchronized (Shooter.this) {
				stop();
			}
		}
		
		@Override
		public void onLoop(double timestamp) {
			synchronized (Shooter.this) {
				
			}
		}
		
		@Override
		public void onStop(double timestamp) {
			synchronized (Shooter.this) {
				stop();
			}
		}
	};
	
	public Loop getLoop() {
		return _loop;
	}
	
	private Shooter() {
		// First Stage Motor
		_firstStgMtr = new CANTalon(Constants.SHOOTER_STG1_CAN_BUS_ADDR);
		_firstStgMtr.changeControlMode(CANTalon.TalonControlMode.Speed);	// open loop throttle
		_firstStgMtr.enableBrakeMode(false);							// default to brake mode DISABLED
    	_firstStgMtr.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);	// set encoder to be feedback device
    	_firstStgMtr.reverseSensor(true);  							// do not invert encoder feedback
		_firstStgMtr.enableLimitSwitch(false, false);
        // set the peak and nominal outputs, 12V means full 
		_firstStgMtr.configNominalOutputVoltage(+0.0f, -0.0f);
		_firstStgMtr.configPeakOutputVoltage(0.0f, -12.0f);
    	
		// set closed loop gains in slot0 
		_firstStgMtr.setProfile(0);
		_firstStgMtr.setF(FIRST_STAGE_MTG_FF_GAIN); 
		_firstStgMtr.setP(FIRST_STAGE_MTG_P_GAIN); 
		_firstStgMtr.setI(FIRST_STAGE_MTG_I_GAIN); 
		_firstStgMtr.setD(FIRST_STAGE_MTG_D_GAIN);
		_firstStgMtr.SetVelocityMeasurementPeriod(VelocityMeasurementPeriod.Period_100Ms);
		//_firstStgMtr.SetVelocityMeasurementWindow(windowSize);
		
		// Second Stage Motor
		_secondStgMtr = new CANTalon(Constants.SHOOTER_STG2_CAN_BUS_ADDR);
		_secondStgMtr.changeControlMode(CANTalon.TalonControlMode.Speed);	// open loop throttle
		_secondStgMtr.enableBrakeMode(false);							// default to brake mode DISABLED
    	_secondStgMtr.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);	// set encoder to be feedback device
    	_secondStgMtr.reverseSensor(true);  							// do not invert encoder feedback
		_secondStgMtr.enableLimitSwitch(false, false);
    	//_secondStageMtr.reverseOutput(true);
        // set the peak and nominal outputs, 12V means full
		_secondStgMtr.configNominalOutputVoltage(+0.0f, -0.0f);
		_secondStgMtr.configPeakOutputVoltage(0.0f, -12.0f);
		
		// set closed loop gains in slot0
		_secondStgMtr.setProfile(0);
		_secondStgMtr.setF(SECOND_STAGE_MTG_FF_GAIN); 
		_secondStgMtr.setP(SECOND_STAGE_MTG_P_GAIN); 
		_secondStgMtr.setI(SECOND_STAGE_MTG_I_GAIN); 
		_secondStgMtr.setD(SECOND_STAGE_MTG_D_GAIN);
		
		// Slider
		_linearActuator = new Servo(Constants.SHOOTER_SLIDER_PWM_PORT);
		
		// default to bumping rmp up on both shooter motors
		_isStg1MtrTargetRPMBumpingUp = true;
		_isStg2MtrTargetRPMBumpingUp = true;
		
		// setup the shooter table
		_shooterTable = ShooterTable.getInstance();
		_currentShooterTableEntry = _shooterTable.getCurrentEntry();
	}
	
	public synchronized void setTargetDistanceInInches(double inches) {
		_currentShooterTableEntry = _shooterTable.CalcShooterValues2(inches);
	}
	
	public synchronized void runAtTargetSpeed() {
		if(Math.abs(_stg2MtrTargetRPM) == 0) {
			runStg2(_currentShooterTableEntry.Stg2MotorRPM);
			setActuatorPosition(_currentShooterTableEntry.SliderPosition);
		}
		else if ((Math.abs(getStg2RPMErrorPercent()) < 7.50) && (Math.abs(_stg1MtrTargetRPM) == 0)) {
			runStg1(_currentShooterTableEntry.Stg1MotorRPM);
		}
		else if(Math.abs(getStg1RPMErrorPercent()) < 5.0 ) {
			// dynamically adjust speeds if current table entry is changed
			if(_lastShooterTableEntry.Index != _currentShooterTableEntry.Index) {
				runStg2(_currentShooterTableEntry.Stg2MotorRPM);
				runStg1(_currentShooterTableEntry.Stg1MotorRPM);
				// allow user to override slider if we have not chg to a new index in the shooter table
				setActuatorPosition(_currentShooterTableEntry.SliderPosition);
			}
		}
		
		// cache last so we can tell if we changed
		_lastShooterTableEntry = _currentShooterTableEntry;
	}
	
	public synchronized void IndexShooterTableUp() {
		_currentShooterTableEntry = _shooterTable.getNextEntry();
	}
	
	public synchronized void IndexShooterTableDown() {
		_currentShooterTableEntry = _shooterTable.getPreviousEntry();
	}
	
	private synchronized void runStg1(double targetRPM) {
		_stg1MtrTargetRPM = targetRPM;
		_firstStgMtr.set(_stg1MtrTargetRPM);
		DriverStation.reportWarning("Stage 1 Target RPM = " + targetRPM, false);
	}
	
	private synchronized void runStg2(double targetRPM) {
		_stg2MtrTargetRPM = targetRPM;
		_secondStgMtr.set(_stg2MtrTargetRPM);
		DriverStation.reportWarning("Stage 2 Target RPM = " + targetRPM, false);
	}
	
	public synchronized void BumpStg1MtrRPMUp() {
		// only bump if not already at max
		if(Math.abs(_stg1MtrTargetRPM) <= Math.abs(MAX_SHOOTER_RPM)) {	
			runStg1(_stg1MtrTargetRPM -= SHOOTER_BUMP_RPM);
		} else {
			DriverStation.reportWarning("Stg 1 Mtr Already at MAX ", false);
			_isStg1MtrTargetRPMBumpingUp = false;
		}
	}
	
	public synchronized void BumpStg1MtrRPMDown() {
		// only bump if not already at min
		if(Math.abs(_stg1MtrTargetRPM) > Math.abs(MIN_SHOOTER_RPM)) {
			runStg1(_stg1MtrTargetRPM += SHOOTER_BUMP_RPM);
		} else {
			DriverStation.reportWarning("Stg 1 Mtr Already at MIN ", false);
			_isStg1MtrTargetRPMBumpingUp = true;
		}
	}
	
	public synchronized void BumpStg2MtrRPMUp() {
		// only bump if not already at max
		if(Math.abs(_stg2MtrTargetRPM) <= Math.abs(MAX_SHOOTER_RPM)) {	
			runStg2(_stg2MtrTargetRPM -= SHOOTER_BUMP_RPM);
		} else {
			DriverStation.reportWarning("Stg 2 Mtr Already at MAX ", false);
			_isStg2MtrTargetRPMBumpingUp = false;
		}
	}

	public synchronized void BumpStg2MtrRPMDown() {
		// only bump if not already at min
		if(Math.abs(_stg2MtrTargetRPM) > Math.abs(MIN_SHOOTER_RPM)) {
			runStg2(_stg2MtrTargetRPM += SHOOTER_BUMP_RPM);
		} else {
			DriverStation.reportWarning("Stg 2 Mtr Already at MIN ", false);
			_isStg2MtrTargetRPMBumpingUp = true;
		}
	}
	
	public synchronized void setActuatorPosition(double position) {
		_currentSliderPosition = position;
		_linearActuator.setPosition(_currentSliderPosition);
	}
	
	public synchronized void moveActuatorUp() {
		// are we below the max?
		if (_currentSliderPosition < MAX_THRESHOLD_ACTUATOR) {
			// increment the target position
			_currentSliderPosition += CHANGE_INTERVAL_ACTUATOR;
			
			// protect fwd limit
			if(_currentSliderPosition > MAX_THRESHOLD_ACTUATOR) {
				_currentSliderPosition = MAX_THRESHOLD_ACTUATOR;
			}
			
			//rounds to 3 Decimal Places
			_currentSliderPosition = GeneralUtilities.RoundDouble(_currentSliderPosition, 3); 
			
			// actually move the slider
			_linearActuator.setPosition(_currentSliderPosition);
			
			DriverStation.reportWarning("Actuator Move Up To: " + _currentSliderPosition, false);
		} else {
			DriverStation.reportWarning("!=!=!=!=! Actuator Already at MAXIMUM Position", false);
		}
	}
	
	public synchronized void moveActuatorDown() {
		// are we above the min?
		if (_currentSliderPosition > MIN_THRESHOLD_ACTUATOR) {
			// decrement the target position
			_currentSliderPosition -= CHANGE_INTERVAL_ACTUATOR;
			
			// protect fwd limit
			if(_currentSliderPosition < MIN_THRESHOLD_ACTUATOR) {
				_currentSliderPosition = MIN_THRESHOLD_ACTUATOR;
			}
			
			//rounds to 3 Decimal Places
			_currentSliderPosition = GeneralUtilities.RoundDouble(_currentSliderPosition, 3); 
			
			// actually move the slider
			_linearActuator.setPosition(_currentSliderPosition);
			
			DriverStation.reportWarning("Actuator Move Down To: " + _currentSliderPosition, false);
		} else {
			DriverStation.reportWarning("!=!=!=!=! Actuator Already at MINIMUM Position", false);
		}
	}
	
	private synchronized double getStg1RPMErrorPercent() {
		if(Math.abs(_stg1MtrTargetRPM) > 0 ) {		
			return ((_stg1MtrTargetRPM - _firstStgMtr.getSpeed()) / _stg1MtrTargetRPM) * 100.0 * -1.0;
		} else {
			return 0.0;
		}
	}
	
	private synchronized double getStg1CurrentPercentVBus() {
		double currentOutputVoltage = _firstStgMtr.getOutputVoltage();
		double currentBusVoltage = _firstStgMtr.getBusVoltage();
		
		double currentActualSpeed = (currentOutputVoltage / currentBusVoltage);
		
		return GeneralUtilities.RoundDouble(currentActualSpeed, 2);
	}
	
	private synchronized double getStg2RPMErrorPercent() {
		if(Math.abs(_stg2MtrTargetRPM) > 0 ) {		
			return ((_stg2MtrTargetRPM - _secondStgMtr.getSpeed()) / _stg2MtrTargetRPM) * 100.0 * -1.0;
		} else {
			return 0.0;
		}
	}
	
	private synchronized double getStg2CurrentPercentVBus() {
		double currentOutputVoltage = _secondStgMtr.getOutputVoltage();
		double currentBusVoltage = _secondStgMtr.getBusVoltage();
		
		double currentActualSpeed = (currentOutputVoltage / currentBusVoltage);
		
		return GeneralUtilities.RoundDouble(currentActualSpeed, 2);
	}

	@Override
	public synchronized void stop() {
		_firstStgMtr.set(0.0);
		_secondStgMtr.set(0.0);
		_linearActuator.set(INITIAL_POSITION_ACTUATOR);
	}

	@Override
	public void zeroSensors() {	
	}

	@Override
	public void outputToSmartDashboard() {
		//%s - insert a string
		//%d - insert a signed integer (decimal)
		//%f - insert a real number, standard notation
		
		// working varibles
		String stg1MtrData = "?";
		String stg2MtrData = "?";
		String actuatorData = "?";
		String currentShooterTableValues = "?";
		
		//Display Current Shooter Motor 1 & 2  | Target | Actual RPM | Error | %Out
		stg1MtrData = String.format("[%.0f RPM] %.0f RPM (%.2f%%) [%.0f%%]", 
												-1 * _stg1MtrTargetRPM, 
												-1 * _firstStgMtr.getSpeed(), 
												getStg1RPMErrorPercent(),
												getStg1CurrentPercentVBus() * 100);
		
		stg2MtrData = String.format("[%.0f RPM] %.0f RPM (%.2f%%) [%.0f%%]", 
												-1 * _stg2MtrTargetRPM, 
												-1 * _secondStgMtr.getSpeed(), 
												getStg2RPMErrorPercent(),
												getStg2CurrentPercentVBus() * 100);
		
		SmartDashboard.putString("Stg 1 RPM Target|Act|(Err%)|[%Out]", stg1MtrData);
		SmartDashboard.putString("Stg 2 RPM Target|Act|(Err%)|[%Out]", stg2MtrData);
		
		//Display Current Actuator Value
		actuatorData = String.format( "%.3f", _currentSliderPosition); //Outputs "Max" and "Min" at respective values
		
		if(_currentSliderPosition >= MAX_THRESHOLD_ACTUATOR) {
			actuatorData = actuatorData + " (MAX)";
		}
		else if(_currentSliderPosition <= MIN_THRESHOLD_ACTUATOR) {
			actuatorData = actuatorData + " (MIN)";
		}
		
		SmartDashboard.putString("Actuator Position", actuatorData);
		String suffix = "";
		if(_shooterTable.get_IsAtLowerEntry()) {
			suffix = "(1st)";
		}
		else if (_shooterTable.get_IsAtUpperEntry()) {
			suffix = "(Last)";
		}
		
		// currentShooterTableValues
		currentShooterTableValues = String.format("[#%d] %s | In:%.1f  |S:%.3f | M1:%d RPM | M2:%d RPM | %s", 
				_currentShooterTableEntry.Index,
				suffix,
				_currentShooterTableEntry.DistanceInInches,
				_currentShooterTableEntry.SliderPosition,
				_currentShooterTableEntry.Stg1MotorRPM,
				_currentShooterTableEntry.Stg2MotorRPM,
				_currentShooterTableEntry.Description);
		
		
	    	// limit spamming
	    	if((new Date().getTime() - _lastDebugWriteTimeMSec) > 1000) {
	    		System.out.println(currentShooterTableValues);
	    		// reset last time
	    		_lastDebugWriteTimeMSec = new Date().getTime();
	    	}

		
		SmartDashboard.putString("ShooterTable", currentShooterTableValues);
		SmartDashboard.putString("Distance", _currentShooterTableEntry.Description);
		
		// Light will come on when the shooter is turned on. To keep Mikey from running it the whole match!! :) 
		SmartDashboard.putBoolean("Is Shooter Running?", _isShooterMotorsReentrantRunning);
	}

	@Override
	public void updateLogData(LogData logData) {
		logData.AddData("Stg1Mtr:Cmd_Rpm", String.format("%f", _stg1MtrTargetRPM));
		logData.AddData("Stg1Mtr:Act_Rpm", String.format("%f", _firstStgMtr.getSpeed()));
		logData.AddData("Stg1Mtr:Err_%", String.format("%.2f%%", getStg1RPMErrorPercent()));
		logData.AddData("Stg1Mtr:%VBus", String.format("%.2f%%", getStg1CurrentPercentVBus()));
			
		logData.AddData("Stg2Mtr:Cmd_Rpm", String.format("%f", _stg2MtrTargetRPM));
		logData.AddData("Stg2Mtr:Act_Rpm", String.format("%f", _secondStgMtr.getSpeed()));	
		logData.AddData("Stg2Mtr:Err_%", String.format("%.2f%%", getStg2RPMErrorPercent()));
		logData.AddData("Stg2Mtr:%VBus", String.format("%.2f%%", getStg2CurrentPercentVBus()));

		logData.AddData("Actuator Position", String.format("%.3f", _currentSliderPosition));
	}
}