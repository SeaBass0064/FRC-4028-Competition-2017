package org.usfirst.frc.team4028.robot.subsystems;

import org.usfirst.frc.team4028.robot.Constants;
import org.usfirst.frc.team4028.util.LogData;
import org.usfirst.frc.team4028.util.loops.Loop;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;

// Feeds balls into the shooter
public class Hopper extends Subsystem {
	private static Hopper _instance = new Hopper();
	public static Hopper getInstance() {
		return _instance;
	}
	
	private CANTalon _magicCarpetMtr, _highSpeedInfeedLaneMtr, _highRollerMtr;
	private Servo _hopperCarousel, _hopperServo;
	
	private double _hopperCarouselReentrantRunningSec;
	private HOPPER_STATE _currentState, _lastState;
	
	private static final double MAGIC_CARPET_TARGET_PERCENTVBUS_COMMAND = -0.7;
	private static final double HIGH_ROLLER_TARGET_PERCENTVBUS_COMMAND = 0.7;
	private static final double HIGH_SPEED_INFEED_LANE_TARGET_PERCENTVBUS_COMMAND = -0.85;
	
	public enum HOPPER_STATE {
		FORWARD,
		REVERSE,
		STOP
	}
	
	private final Loop _loop = new Loop() {
		@Override
		public void onStart(double timestamp) {
			synchronized (Hopper.this) {	
				stop();
				_currentState = HOPPER_STATE.STOP;
			}
		}
		
		@Override
		public void onLoop(double timestamp) {
			synchronized (Hopper.this) {
				switch (_currentState) {
					case FORWARD:
						if (_currentState != _lastState) {
							_hopperCarouselReentrantRunningSec = Timer.getFPGATimestamp();
						}
						runHopperCarousel();
						break;
						
					case REVERSE:
						runInReverse();
						break;
						
					case STOP:
						stop();
						break;
				}
				_lastState = _currentState;
			}
		}
		
		@Override
		public void onStop(double timestamp) {
			synchronized (Hopper.this) {	
				stop();
			}
		}
	};
	
	public Loop getLoop() {
		return _loop;
	}
	
	private Hopper() {
		// Magic Carpet Motor
		_magicCarpetMtr = new CANTalon(Constants.MAGIC_CARPET_CAN_BUS_ADDR);
		_magicCarpetMtr.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		_magicCarpetMtr.enableBrakeMode(false);
		_magicCarpetMtr.enableLimitSwitch(false, false);
		
		// High Speed Infeed Lane Motor
		_highSpeedInfeedLaneMtr = new CANTalon(Constants.HIGH_SPEED_INFEED_LANE_CAN_BUS_ADDR);
		_highSpeedInfeedLaneMtr.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		_highSpeedInfeedLaneMtr.enableBrakeMode(false);
		_highSpeedInfeedLaneMtr.enableLimitSwitch(false, false);
		
		// High Roller Motor
		_highRollerMtr = new CANTalon(Constants.HIGH_ROLLER_CAN_BUS_ADDR);
		_highRollerMtr.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		_highRollerMtr.enableBrakeMode(false);
		_highRollerMtr.enableLimitSwitch(false, false);
		
		// Hopper Carousel
		_hopperCarousel = new Servo(8);
		_hopperServo = new Servo(7);
	}
	
	public synchronized void setState(HOPPER_STATE state) {
		_currentState = state;
	}
	
	private synchronized void runInReverse() {
		_magicCarpetMtr.set(-1.0 * MAGIC_CARPET_TARGET_PERCENTVBUS_COMMAND);
		_highSpeedInfeedLaneMtr.set(-1.0 * HIGH_SPEED_INFEED_LANE_TARGET_PERCENTVBUS_COMMAND);
		_highRollerMtr.set(-1.0 * HIGH_ROLLER_TARGET_PERCENTVBUS_COMMAND);
	}
	
	private synchronized void setHopperServos(double position) {
		_hopperCarousel.setPosition(position);
		_hopperServo.setPosition(position);
	}
	
	private synchronized void runHopperCarousel() {
		if ((Timer.getFPGATimestamp() - _hopperCarouselReentrantRunningSec) < 0.75) {
			setHopperServos(0.0);
		} else if ((Timer.getFPGATimestamp() - _hopperCarouselReentrantRunningSec) < 1.5) {
			setHopperServos(1.0);
		} else {
			_hopperCarouselReentrantRunningSec = Timer.getFPGATimestamp();
		}
	}

	@Override
	public void stop() {
		_magicCarpetMtr.set(0.0);
		_highSpeedInfeedLaneMtr.set(0.0);
		_highRollerMtr.set(0.0);
		setHopperServos(0.0);
	}

	@Override
	public void zeroSensors() {}

	@Override
	public void outputToSmartDashboard() {}

	@Override
	public void updateLogData(LogData logData) {}
}