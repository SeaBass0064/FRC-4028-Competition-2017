package org.usfirst.frc.team4028.robot.subsystems;

import org.usfirst.frc.team4028.robot.Constants;
import org.usfirst.frc.team4028.util.LogData;
import org.usfirst.frc.team4028.util.loops.Loop;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Infeed extends Subsystem {
	private static Infeed _instance = new Infeed();
	public static Infeed getInstance() {
		return _instance;
	}
	
	private CANTalon _infeedMtr;
	private Solenoid _infeedSolenoid;
	
	private final Loop _loop = new Loop() {
		@Override
		public void onStart(double timestamp) {
			stop();
		}
		
		@Override
		public void onLoop(double timestamp) {
			synchronized (Infeed.this) {		
			}
		}
		
		@Override
		public void onStop(double timestamp) {
			stop();
		}
	};
	
	public Loop getLoop() {
		return _loop;
	}
	
	private Infeed() {
		_infeedMtr = new CANTalon(Constants.BALL_FLOOR_INFEED_MTR_CAN_BUS_ADDR);
		_infeedMtr.changeControlMode(TalonControlMode.PercentVbus);
		_infeedMtr.enableBrakeMode(false);
		_infeedMtr.enableLimitSwitch(false, false);
		
		_infeedSolenoid = new Solenoid(Constants.PCM_CAN_BUS_ADDR, Constants.BALL_FLOOR_INFEED_EXTEND_PCM_PORT);
	}
	
	public synchronized void infeedFuel(double percentVBusCmd) {
		_infeedMtr.set(percentVBusCmd * -1.0);
		
		// if running fwd or reverse fire solenoid
				if(percentVBusCmd != 0) {
					_infeedSolenoid.set(true);			//extend Solenoid
				} else {
					_infeedSolenoid.set(false);			//retract Solenoid
				}
	}

	@Override
	public synchronized void stop() {
		_infeedMtr.set(0);
		_infeedSolenoid.set(false);
	}

	@Override
	public void zeroSensors() {}

	@Override
	public void outputToSmartDashboard() {
		SmartDashboard.putBoolean("Is Fuel Infeed Tilt Extended", _infeedSolenoid.get());
		
		String ballInfeedMtrData = "?";
		
		if(Math.abs(_infeedMtr.getOutputVoltage()) > 0) {
			ballInfeedMtrData = String.format("%s (%.0f%%)", 
												"ON", 
												(_infeedMtr.getOutputVoltage() / _infeedMtr.getBusVoltage())* 100);
		} else {
			ballInfeedMtrData = String.format("%s (%.0f%%)", "off", 0.0);
		}
		
		SmartDashboard.putString("Fuel Infeed", ballInfeedMtrData);
	}

	@Override
	public void updateLogData(LogData logData) {}
}