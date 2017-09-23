package org.usfirst.frc.team4028.robot.vision;

// DTO (data transfer object) This class represents the data retrieved from the RoboRealm API
public class RawImageData {
	public boolean isVisionDataValid;
	
	public int badDataCounter;
	public int blobCount;
	
	public double fovCenterToTargetXAngleRawDegrees;
	public double estimatedDistance;
	public double highMiddleY;
	public double responseTimeMSec;
	public double southWestX, southWestY, southEastX, southEastY;
	
	public long timeStamp;
	
	public Dimension fovDimensions;
	
	public String cameraType;
}