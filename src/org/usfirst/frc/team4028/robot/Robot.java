package org.usfirst.frc.team4028.robot;

import java.math.BigDecimal;
import java.text.DecimalFormat;
import java.util.Date;

import org.usfirst.frc.team4028.robot.GeneralEnums.TELEOP_MODE;
import org.usfirst.frc.team4028.robot.auton.AutonExecuter;
import org.usfirst.frc.team4028.robot.paths.profiles.PathAdapter;
import org.usfirst.frc.team4028.robot.sensors.RoboRealmClient;
import org.usfirst.frc.team4028.robot.sensors.SwitchableCameraServer;
import org.usfirst.frc.team4028.robot.subsystems.Chassis;
import org.usfirst.frc.team4028.robot.subsystems.Climber;
import org.usfirst.frc.team4028.robot.subsystems.GearHandler;
import org.usfirst.frc.team4028.robot.subsystems.GearHandler.GEAR_TILT_STATE;
import org.usfirst.frc.team4028.robot.subsystems.Hopper;
import org.usfirst.frc.team4028.robot.subsystems.Infeed;
import org.usfirst.frc.team4028.robot.subsystems.Shooter;
import org.usfirst.frc.team4028.robot.subsystems.Chassis.GearShiftPosition;
import org.usfirst.frc.team4028.util.DataLogger;
import org.usfirst.frc.team4028.util.GeneralUtilities;
import org.usfirst.frc.team4028.util.LogData;
import org.usfirst.frc.team4028.util.MovingAverage;
import org.usfirst.frc.team4028.util.ShooterTable;
import org.usfirst.frc.team4028.util.loops.Looper;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {
	private static final String ROBOT_NAME = "COMPETITION Chassis";
	
	// Subsystems
	private Chassis _chassis = Chassis.getInstance();
	private Climber _climber = Climber.getInstance();
	private GearHandler _gearHandler = GearHandler.getInstance();
	private Infeed _infeed = Infeed.getInstance();
	private Shooter _shooter = Shooter.getInstance();
	private Hopper _hopper = Hopper.getInstance();
	
	// Sensors
	private RoboRealmClient _roboRealm = RoboRealmClient.getInstance();
	private SwitchableCameraServer _cameraServer = SwitchableCameraServer.getInstance();
	private RobotState _robotState = RobotState.getInstance();
	
	// Other
	private ControlBoard _controlBoard = ControlBoard.getInstance();
	private AutonExecuter _autonExecuter = null;
	private SmartDashboardInputs _smartDashboard = SmartDashboardInputs.getInstance();
	private DataLogger _dataLogger = null;
	
	Looper _enabledLooper = new Looper();
	
	// Class Level Instance Variables
	private TELEOP_MODE _teleopMode;
	
	String _buildMsg = "?";
	ShooterTable _shooterTable;
	String _fmsDebugMsg = "?";
 	long _lastDashboardWriteTimeMSec;
 	long _lastScanEndTimeInMSec;
 	MovingAverage _scanTimeSamples;
	
	@Override
	public void robotInit() {
		_buildMsg = GeneralUtilities.WriteBuildInfoToDashboard(ROBOT_NAME);
		
		_enabledLooper.register(_chassis.getLoop());
		_enabledLooper.register(_climber.getLoop());
		_enabledLooper.register(_gearHandler.getLoop());
		_enabledLooper.register(_infeed.getLoop());
		_enabledLooper.register(_shooter.getLoop());
		_enabledLooper.register(RobotStateEstimator.getInstance().getLoop());
		
		_smartDashboard.printStartupMessage();
		
		// create class to hold Scan Times moving Average samples
		_scanTimeSamples = new MovingAverage(100);  // 2 sec * 1000mSec/Sec / 20mSec/Scan
		SmartDashboard.putString("Scan Time (2 sec roll avg)", "0.0 mSec");
		
		PathAdapter.calculatePaths();
		
		outputAllToSmartDashboard();
	}

	@Override
	public void disabledInit() {
		if (_autonExecuter != null) {
			_autonExecuter.stop();
		}
		_autonExecuter = null;
		
		_enabledLooper.stop();
		stopAll();
	}

	@Override
	public void disabledPeriodic() {
		if (_dataLogger != null) {
			_dataLogger.close();
			_dataLogger = null;
		}
		stopAll();
	}
	
	@Override
	public void autonomousInit() {
		if (_autonExecuter != null) {
			_autonExecuter.stop();
		}
		_autonExecuter = null;
		
		_enabledLooper.start();
		
		_autonExecuter = new AutonExecuter();
		_autonExecuter.setAutoMode(_smartDashboard.getSelectedAuton());
		_autonExecuter.start();
		
		_roboRealm.TurnAllVisionLEDsOff();
		
		_dataLogger = GeneralUtilities.setupLogging("auton");
		
		_lastDashboardWriteTimeMSec = new Date().getTime();
	}

	@Override
	public void autonomousPeriodic() {	
		logAllData();
		outputAllToSmartDashboard();
	}

	@Override
	public void teleopInit() {
		if (_autonExecuter != null) {
			_autonExecuter.stop();
		}
		_autonExecuter = null;
		
		_enabledLooper.start();
		
		stopAll();
		
		_chassis.ShiftGear(GearShiftPosition.HIGH_GEAR);
		
		_dataLogger = GeneralUtilities.setupLogging("teleop");
	}

	@Override
	public void teleopPeriodic() {
		// Switch Between Cameras
		if (_controlBoard.getIsSwapCameraJustPressed()) {
			_cameraServer.ChgToNextCamera();
		}
		
		switch (_teleopMode) {
			case STANDARD:
				// Chassis Throttle & Turn
				if ((Math.abs(_controlBoard.getThrottleCmd()) > 0.0) 
						|| (Math.abs(_controlBoard.getTurnCmd()) > 0.0)) {
					_chassis.arcadeDrive(-1.0 * _controlBoard.getThrottleCmd(), _controlBoard.getTurnCmd());
				} 
				else if ((Math.abs(_controlBoard.getSpinChassisLeftCmd()) > 0.0)
							&& (Math.abs(_controlBoard.getSpinChassisRightCmd()) == 0.0)) {
					// spin left
					_chassis.arcadeDrive(0.0, _controlBoard.getSpinChassisRightCmd() * 0.75 * -1.0);
				} 
				else if ((Math.abs(_controlBoard.getSpinChassisRightCmd()) > 0.0) 
							&& (Math.abs(_controlBoard.getSpinChassisLeftCmd()) == 0.0)) {
					// spin right
					_chassis.arcadeDrive(0.0, _controlBoard.getSpinChassisRightCmd() * 0.75);
				} 
				else if (_controlBoard.getIsVisionAimPressed()) {
					// Turn on auto aiming with vision (for boiler)
					//_chassisAutoAimGyro.motionMagicMoveToTarget(_navX.getYaw() - (_roboRealmClient.get_Angle()/3.5));
				} else {
					_chassis.stop();
				}
				
				// Chassis Gear Shift
				if (_controlBoard.getIsShiftGearJustPressed()) {
					_chassis.ToggleShiftGear();
				}
				
				// Infeed
				_infeed.infeedFuel(_controlBoard.getFuelInfeedOutfeedJoystickCmd());
				
				// Gear Tilt
				if (_controlBoard.getIsRezeroGearTiltPressed()) {
					_gearHandler.setState(GEAR_TILT_STATE.HOMING);
				}
				else if (_controlBoard.getIsSendGearTiltToHomePressed()) {
					_gearHandler.setState(GEAR_TILT_STATE.HOLD_HOME_POSITION);
				}
				else if (_controlBoard.getIsSendGearTiltToScorePressed()) {
					_gearHandler.setState(GEAR_TILT_STATE.HOLD_SCORING_POSITION);
				}
				else if (_controlBoard.getIsSendGearTiltToFloorPressed()) {
					_gearHandler.setState(GEAR_TILT_STATE.HOLD_FLOOR_POSITION);
				}
				else if (_controlBoard.getIsStartGearScoreSequencePressed()) {
					_gearHandler.setState(GEAR_TILT_STATE.RUNNING_HANG_GEAR);
					_teleopMode = TELEOP_MODE.HANG_GEAR_SEQUENCE_MODE;
				}
				
				// Gear Infeed
				if (_controlBoard.getIsInfeedGearPressed()) {
					_gearHandler.spinInfeedWheels(1.0);
				}
				else if (_controlBoard.getIsOutfeedGearPressed()) {
					_gearHandler.spinInfeedWheels(-1.0);
				}
				else {
					_gearHandler.spinInfeedWheels(0.0);
				}
				break;
				
			case HANG_GEAR_SEQUENCE_MODE:	// Sequence for hanging gear
				if (_gearHandler.getState() == GEAR_TILT_STATE.HOLD_SCORING_POSITION) {
					_teleopMode = TELEOP_MODE.STANDARD;
				} else {
					_chassis.arcadeDrive(-0.6, 0.0);
				}
				break;
		}
		
		// Climber
		_climber.runMotor(_controlBoard.getClimberSpeedCmd());
		
		// Refresh Dashboard
		outputAllToSmartDashboard();
		
		// Optionally Log Data
		logAllData();
	}
	
	private void stopAll() {
		_chassis.stop();
		_climber.stop();
		_gearHandler.stop();
		_infeed.stop();
		_shooter.stop();
		_hopper.stop();
	}
	
	private void outputAllToSmartDashboard() {
		// limit spamming
    	long scanCycleDeltaInMSecs = new Date().getTime() - _lastScanEndTimeInMSec;
    	_scanTimeSamples.add(new BigDecimal(scanCycleDeltaInMSecs));
    	
    	if((new Date().getTime() - _lastDashboardWriteTimeMSec) > 100) {
    		_chassis.outputToSmartDashboard();
    		_climber.outputToSmartDashboard();
    		_gearHandler.outputToSmartDashboard();
    		_infeed.outputToSmartDashboard();
    		_hopper.outputToSmartDashboard();
    		_shooter.outputToSmartDashboard();
	    	
    		_cameraServer.outputToSmartDashboard();
	    	
	    	_roboRealm.outputToSmartDashboard(); 
	    	
	    	SmartDashboard.putString("Robot Build", _buildMsg);
	    	SmartDashboard.putString("FMS Debug Msg", _fmsDebugMsg);
	    	
	    	BigDecimal movingAvg = _scanTimeSamples.getAverage();
	    	DecimalFormat df = new DecimalFormat("####");
	    	SmartDashboard.putString("Scan Time (2 sec roll avg)", df.format(movingAvg) + " mSec");
	    	
    		// snapshot last time
    		_lastDashboardWriteTimeMSec = new Date().getTime();
    	}
    	
    	// snapshot when this scan ended
    	_lastScanEndTimeInMSec = new Date().getTime();
	}
	
	private void logAllData() {
		// always call this 1st to calc drive metrics
    	if(_chassis != null) 			{ _chassis.UpdateDriveTrainPerfMetrics(); }
    	
    	if(_dataLogger != null) {    	
	    	// create a new, empty logging class
        	LogData logData = new LogData();
	    	
	    	// ask each subsystem that exists to add its data
        	// 23.Apr.2017 TomB Commented most out to improve logging perf
	    	_chassis.updateLogData(logData);
	    	//_climber.updateLogData(logData);
	    	//_gearHandler.updateLogData(logData);
	    	//_infeed.updateLogData(logData);
	    	_shooter.updateLogData(logData);
	    	//_hopper.updateLogData(logData);
	    	
	    	_roboRealm.updateLogData(logData);
	    	
	    	// now write to the log file
	    	_dataLogger.WriteDataLine(logData);
    	}
	}
}