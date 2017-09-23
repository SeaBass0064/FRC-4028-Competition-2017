package org.usfirst.frc.team4028.robot;

import org.usfirst.frc.team4028.robot.subsystems.Chassis;
import org.usfirst.frc.team4028.util.loops.Loop;
import org.usfirst.frc.team4028.util.motion.Rotation;
import org.usfirst.frc.team4028.util.motion.Twist;

/**
 * Periodically estimates the state of the robot using the robot's distance traveled (compares two waypoints), gyroscope
 * orientation, and velocity, among various other factors. Similar to a car's odometer.
 */
public class RobotStateEstimator {
	static RobotStateEstimator instance_ = new RobotStateEstimator();

    public static RobotStateEstimator getInstance() {
        return instance_;
    }

    RobotStateEstimator() {
    }

    RobotState robot_state_ = RobotState.getInstance();
    Chassis _chassis = Chassis.getInstance();
    double left_encoder_prev_distance_ = 0;
    double right_encoder_prev_distance_ = 0;
    
    private final Loop _loop = new Loop() {
	    @Override
	    public synchronized void onStart(double timestamp) {
	    	synchronized(RobotStateEstimator.this) {
		        left_encoder_prev_distance_ = _chassis.getLeftDistanceInches();
		        right_encoder_prev_distance_ = _chassis.getRightDistanceInches();
	    	}
	    }
	
	    @Override
	    public synchronized void onLoop(double timestamp) {
	    	synchronized(RobotStateEstimator.this) {
		        final double left_distance = _chassis.getLeftDistanceInches();
		        final double right_distance = _chassis.getRightDistanceInches();
		        final Rotation gyro_angle = Rotation.fromDegrees(_chassis.getHeading());
		        final Twist odometry_velocity = robot_state_.generateOdometryFromSensors(
		                left_distance - left_encoder_prev_distance_, right_distance - right_encoder_prev_distance_, gyro_angle);
		        final Twist predicted_velocity = Kinematics.forwardKinematics(_chassis.getLeftVelocityInchesPerSec(),
		                _chassis.getRightVelocityInchesPerSec());
		        robot_state_.addObservations(timestamp, odometry_velocity, predicted_velocity);
		        left_encoder_prev_distance_ = left_distance;
		        right_encoder_prev_distance_ = right_distance;
	    	}
	    }
	
	    @Override
	    public void onStop(double timestamp) {
	    	synchronized(RobotStateEstimator.this) {
	    		// no-op
	    	}
	    }
    };
    
    public Loop getLoop() {
    	return _loop;
    }
}