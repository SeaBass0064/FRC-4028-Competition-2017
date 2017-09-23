package org.usfirst.frc.team4028.robot.subsystems;

import org.usfirst.frc.team4028.robot.Constants;
import org.usfirst.frc.team4028.util.LogData;
import org.usfirst.frc.team4028.util.loops.Loop;

import com.ctre.CANTalon;

public class Climber extends Subsystem{
	private static Climber _instance = new Climber();
	public static Climber getInstance() {
		return _instance;
	}
	
	private CANTalon _climberMtr;
	
	// define class level constants
	public static final double CLIMBER_MOTOR_HIGH_VBUS = -1.0;
	public static final double CLIMBER_MOTOR_LOW_VBUS = -0.40;
	
	private final Loop _loop = new Loop() {
		@Override
		public void onStart(double timestamp) {
			synchronized (Climber.this) {
				stop();
			}
		}
		
		@Override
		public void onLoop(double timestamp) {
			synchronized (Climber.this) {}
		}
		
		@Override
		public void onStop(double timestamp) {
			synchronized (Climber.this) {
				stop();
			}
		}
	};
	
	public Loop getLoop() {
		return _loop;
	}
	
	private Climber() {
		_climberMtr = new CANTalon(Constants.CLIMBER_CAN_BUS_ADDR);
		_climberMtr.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		_climberMtr.enableBrakeMode(true);
		_climberMtr.enableLimitSwitch(false, false);
	}
	
	public synchronized void runMotor(double joystickCmd) {
		if(joystickCmd < -0.8) {
			// more than 1/2 way, climb @ high speed
			_climberMtr.set(CLIMBER_MOTOR_HIGH_VBUS);
		}
		else if(joystickCmd < -0.05) {
			// more than 1/2 way, climb @ low speed
			_climberMtr.set(CLIMBER_MOTOR_LOW_VBUS);
		}
		else {
			_climberMtr.set(0.0);
		}
	}
	
	@Override
	public synchronized void stop() {
		_climberMtr.set(0.0);
	}
	
	@Override
	public void zeroSensors() {}
	@Override
	public void outputToSmartDashboard() {
		
	}
	@Override
	public void updateLogData(LogData logData) {
		
	}
}