package org.usfirst.frc.team4028.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

// This class contains id values for the physical elements of the robot so we can use names in the code instead of hardcoded constants
public class Constants {
	// Gamepad USB Port
	public static final int DRIVER_GAMEPAD_USB_PORT = 0;
	public static final int OPERATOR_GAMEPAD_USB_PORT = 1;
	
	// PCM Can Bus
	public static final int PCM_CAN_BUS_ADDR = 0;	
	
	// Talons Can Bus
	public static final int LEFT_DRIVE_MASTER_CAN_BUS_ADDR = 11;
	public static final int LEFT_DRIVE_SLAVE_CAN_BUS_ADDR = 12;
	public static final int RIGHT_DRIVE_MASTER_CAN_BUS_ADDR = 9;
	public static final int RIGHT_DRIVE_SLAVE_CAN_BUS_ADDR = 10;	
	public static final int CLIMBER_CAN_BUS_ADDR = 5;
	public static final int GEAR_TILT_CAN_BUS_ADDR = 7;
	public static final int GEAR_INFEED_CAN_BUS_ADDR = 8;
	public static final int SHOOTER_STG1_CAN_BUS_ADDR = 1;
	public static final int SHOOTER_STG2_CAN_BUS_ADDR = 2;
	public static final int MAGIC_CARPET_CAN_BUS_ADDR = 6;
	public static final int HIGH_SPEED_INFEED_LANE_CAN_BUS_ADDR = 4;
	public static final int HIGH_ROLLER_CAN_BUS_ADDR = 13;
	public static final int BALL_FLOOR_INFEED_MTR_CAN_BUS_ADDR = 3; 
	public static final int EXTRA_MOTOR_BUS_ADDR = 14;
	
	// PWM Ports
	public static final int SHOOTER_SLIDER_PWM_PORT = 9;
	
	// DIO Ports
	public static final int LED_RINGS_DIO_PORT = 9;
	
	// NavX (on Roborio)
	public static final SPI.Port NAVX_PORT = Port.kMXP;
	
	// USB Cameras
	public static final String GEAR_CAMERA_NAME = "cam0";
	public static final String BALL_INFEED_CAMERA_NAME = "cam1";
	public static final String SHOOTER_CAMERA_NAME = "cam2";
	public static final String CLIMBER_CAMERA_NAME = "cam3";
	
	// Solenoid Ports on PCM
	public static final int GEAR_LED_RING_PCM_PORT = 0;
	public static final int BOILER_LED_RING_PCM_PORT = 1;	
	
	public static final int SHIFTER_SOLENOID_RETRACT_PCM_PORT = 7;
	public static final int SHIFTER_SOLENOID_EXTEND_PCM_PORT = 6;
	public static final int BALL_FLOOR_INFEED_EXTEND_PCM_PORT = 5;
	
	// Solenoid Positions
	public static final Value SHIFTER_SOLENOID_LOW_GEAR_POSITION = DoubleSolenoid.Value.kForward;
	public static final Value SHIFTER_SOLENOID_HIGH_GEAR_POSITION = DoubleSolenoid.Value.kReverse;
	
	// Logging
	// this is where the USB stick is mounted on the RoboRIO filesystem.  You can confirm by logging into the RoboRIO using WinSCP
	public static final String PRIMARY_LOG_FILE_PATH = "/media/sda1/logging";
	public static final String ALTERNATE_LOG_FILE_PATH = "/media/sdb1/logging";
	
	// Kangaroo and Roborealm Constants
	public static final String KANGAROO_IPV4_ADDR = "10.40.28.78";
	public static final int RR_API_PORT = 5800;
	
	/* CONTROL LOOP GAINS */

    // PID gains for drive velocity loop (HIGH GEAR)
    // Units: setpoint, error, and output are in inches per second.
    public static double kDriveHighGearVelocityKp = 1.2;
    public static double kDriveHighGearVelocityKi = 0.0;
    public static double kDriveHighGearVelocityKd = 6.0;
    public static double kDriveHighGearVelocityKf = .15;
    public static int kDriveHighGearVelocityIZone = 0;
    public static double kDriveHighGearVelocityRampRate = 240.0;
    public static double kDriveHighGearNominalOutput = 0.5;
    public static double kDriveHighGearMaxSetpoint = 17.0 * 12.0; // 17 fps
    
    public static double kDriveVoltageCompensationRampRate = 0.0;
    
	/* Robot Physical Constants */
	// Wheels
	public static double kDriveWheelDiameterInches = 0.0;
	public static double kTrackWidthInches = 0.0;
	public static double kTrackScrubFactor = 0.0;
	
	// Geometry
	public static double kCenterToFrontBumperDistance = 16.33;
    public static double kCenterToIntakeDistance = 23.11;
    public static double kCenterToRearBumperDistance = 16.99;
    public static double kCenterToSideBumperDistance = 17.225;
	
    
    // Path Following Constants
    public static double kMinLookAhead = 12.0; // inches
    public static double kMinLookAheadSpeed = 9.0; // inches per second
    public static double kMaxLookAhead = 24.0; // inches
    public static double kMaxLookAheadSpeed = 120.0; // inches per second
    public static double kDeltaLookAhead = kMaxLookAhead - kMinLookAhead;
    public static double kDeltaLookAheadSpeed = kMaxLookAheadSpeed - kMinLookAheadSpeed;

    public static double kInertiaSteeringGain = 0.0; // angular velocity command is multiplied by this gain *
                                                     // our speed
                                                     // in inches per sec
    public static double kSegmentCompletionTolerance = 0.1; // inches
    public static double kPathFollowingMaxAccel = 120.0; // inches per second^2
    public static double kPathFollowingMaxVel = 120.0; // inches per second
    public static double kPathFollowingProfileKp = 5.00;
    public static double kPathFollowingProfileKi = 0.03;
    public static double kPathFollowingProfileKv = 0.02;
    public static double kPathFollowingProfileKffv = 1.0;
    public static double kPathFollowingProfileKffa = 0.05;
    public static double kPathFollowingGoalPosTolerance = 0.75;
    public static double kPathFollowingGoalVelTolerance = 12.0;
    public static double kPathStopSteeringDistance = 9.0;
}