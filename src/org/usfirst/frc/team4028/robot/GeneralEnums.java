package org.usfirst.frc.team4028.robot;

public class GeneralEnums {
	public enum TELEOP_MODE {
		STANDARD,
		HANG_GEAR_SEQUENCE_MODE
	}
	
	public enum AUTON_MODE {
		UNDEFINED,
		CROSS_BASE_LINE,
		DO_NOTHING,
		HANG_BOILER_GEAR,
		HANG_BOILER_GEAR_AND_SHOOT,
		HANG_CENTER_GEAR,
		HANG_CENTER_GEAR_AND_SHOOT,
		HANG_RETRIEVAL_GEAR,
		HIT_HOPPER,
		TURN_AND_SHOOT,
		TWO_GEAR
	}
	
	public enum ALLIANCE_COLOR {
		USE_FMS,
		BLUE_ALLIANCE,
		RED_ALLIANCE
	}
	
	public enum CAMERA_NAMES {
		UNDEFINED 	("N/A"),
		CAM0		("cam0"),
		CAM1		("cam1"),
		CAM2		("cam2"),
		CAM3		("cam3");
		
		private final String _cameraName;  
		
		CAMERA_NAMES(String cameraName) {
			_cameraName = cameraName;
		}
		
		public String get_cameraName() {
			return _cameraName;
		}
	}
	
	public enum VISION_CAMERAS {
		UNDEFINED 	("N/A"),
		GEAR		("GEAR"),
		BOILER		("Boiler");
		
		private final String _cameraName;  
		
		VISION_CAMERAS(String cameraName) {
			_cameraName = cameraName;
		}
		
		public String get_cameraName() {
			return _cameraName;
		}
	}
}