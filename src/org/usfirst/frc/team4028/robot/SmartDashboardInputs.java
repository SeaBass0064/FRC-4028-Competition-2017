package org.usfirst.frc.team4028.robot;

import org.usfirst.frc.team4028.robot.GeneralEnums.ALLIANCE_COLOR;
import org.usfirst.frc.team4028.robot.GeneralEnums.AUTON_MODE;
import org.usfirst.frc.team4028.robot.GeneralEnums.CAMERA_NAMES;
import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.auton.modes.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartDashboardInputs {
	private static SmartDashboardInputs _instance = new SmartDashboardInputs();
	public static SmartDashboardInputs getInstance() {
		return _instance;
	}
	
	// Enums
	private AUTON_MODE _autonModeChoice;
	private ALLIANCE_COLOR _allianceColor;
	private CAMERA_NAMES _gearCameraName;
	private CAMERA_NAMES _shooterCameraName;
	private CAMERA_NAMES _climberCameraName;
	private CAMERA_NAMES _driverCameraName;
	
	// Choosers
	private SendableChooser<AUTON_MODE> _autonModeChooser;
	private SendableChooser<ALLIANCE_COLOR> _allianceChooser;
	private SendableChooser<CAMERA_NAMES> _gearCamChooser;
	private SendableChooser<CAMERA_NAMES> _shooterCamChooser;
	private SendableChooser<CAMERA_NAMES> _climberCamChooser;
	private SendableChooser<CAMERA_NAMES> _driverCamChooser;
	
	private String _fmsDebugMsg = "?";
	
	private SmartDashboardInputs() {
		ConfigAutonModeChoosers();
		ConfigCameraChoosers();
	}
	
	private void ConfigAutonModeChoosers() {
		// Auton Mode Chooser
		_autonModeChooser = new SendableChooser<AUTON_MODE>();
		
		_autonModeChooser.addObject("Do Nothing", GeneralEnums.AUTON_MODE.DO_NOTHING);
		_autonModeChooser.addObject("Cross the Base Line", GeneralEnums.AUTON_MODE.CROSS_BASE_LINE);
		_autonModeChooser.addObject("Right Side Gear", GeneralEnums.AUTON_MODE.HANG_BOILER_GEAR);
		//_autonModeChooser.addObject("Hang Gear on the Boiler Side and Shoot", GeneralEnums.AUTON_MODE.HANG_BOILER_GEAR_AND_SHOOT);
		_autonModeChooser.addObject("Center Gear", GeneralEnums.AUTON_MODE.HANG_CENTER_GEAR);
		_autonModeChooser.addObject("Hang Gear in Center and Shoot", GeneralEnums.AUTON_MODE.HANG_CENTER_GEAR_AND_SHOOT);
		_autonModeChooser.addObject("Left Side Gear", GeneralEnums.AUTON_MODE.HANG_RETRIEVAL_GEAR);
		_autonModeChooser.addDefault("Hit the Hopper and Shoot", GeneralEnums.AUTON_MODE.HIT_HOPPER);
		_autonModeChooser.addObject("Turn and Shoot", GeneralEnums.AUTON_MODE.TURN_AND_SHOOT);
		//_autonModeChooser.addObject("Two Gears", GeneralEnums.AUTON_MODE.TWO_GEAR);
		
		SmartDashboard.putData("Auton Mode Chooser", _autonModeChooser);
		
		// Alliance Color Chooser
		_allianceChooser = new SendableChooser<ALLIANCE_COLOR>();
		
		_allianceChooser.addDefault("FMS", GeneralEnums.ALLIANCE_COLOR.USE_FMS);
		_allianceChooser.addObject("Red Alliance", GeneralEnums.ALLIANCE_COLOR.RED_ALLIANCE);
		_allianceChooser.addObject("Blue Alliance", GeneralEnums.ALLIANCE_COLOR.BLUE_ALLIANCE);
		
		SmartDashboard.putData("Alliance Chooser" , _allianceChooser);		
	}
	
	private void ConfigCameraChoosers() {
		// #ALL the Cameras
		_gearCamChooser = new SendableChooser<CAMERA_NAMES>();
		
		_gearCamChooser.addDefault("g.cam0", GeneralEnums.CAMERA_NAMES.CAM0);
		_gearCamChooser.addObject("cam1", GeneralEnums.CAMERA_NAMES.CAM1);
		_gearCamChooser.addObject("cam2", GeneralEnums.CAMERA_NAMES.CAM2);
		_gearCamChooser.addObject("cam3", GeneralEnums.CAMERA_NAMES.CAM3);
		
		SmartDashboard.putData("Gear Camera Chooser", _gearCamChooser);
		
		_shooterCamChooser = new SendableChooser<CAMERA_NAMES>();
		
		_shooterCamChooser.addObject("s.cam0", GeneralEnums.CAMERA_NAMES.CAM0);
		_shooterCamChooser.addDefault("cam1", GeneralEnums.CAMERA_NAMES.CAM1);
		_shooterCamChooser.addObject("cam2", GeneralEnums.CAMERA_NAMES.CAM2);
		_shooterCamChooser.addObject("cam3", GeneralEnums.CAMERA_NAMES.CAM3);
		
		SmartDashboard.putData("Shooter Camera Chooser", _shooterCamChooser);
		
		_climberCamChooser = new SendableChooser<CAMERA_NAMES>();
		
		_climberCamChooser.addObject("c.cam0", GeneralEnums.CAMERA_NAMES.CAM0);
		_climberCamChooser.addObject("cam1", GeneralEnums.CAMERA_NAMES.CAM1);
		_climberCamChooser.addDefault("cam2", GeneralEnums.CAMERA_NAMES.CAM2);
		_climberCamChooser.addObject("cam3", GeneralEnums.CAMERA_NAMES.CAM3);
		
		SmartDashboard.putData("Climber Camera Chooser", _climberCamChooser);
		
		_driverCamChooser = new SendableChooser<CAMERA_NAMES>();
		
		_driverCamChooser.addObject("d.cam0", GeneralEnums.CAMERA_NAMES.CAM0);
		_driverCamChooser.addObject("cam1", GeneralEnums.CAMERA_NAMES.CAM1);
		_driverCamChooser.addObject("cam2", GeneralEnums.CAMERA_NAMES.CAM2);
		_driverCamChooser.addDefault("cam3", GeneralEnums.CAMERA_NAMES.CAM3);
		
		SmartDashboard.putData("Driver Camera Chooser", _driverCamChooser);
	}
	
	public boolean getIsFMSAttached() {
		return DriverStation.getInstance().isFMSAttached();
	}
	
	public AutonBase getSelectedAuton() {
		// Returns the autonBase object associated with the auton selected on the dashboard 
		_autonModeChoice = _autonModeChooser.getSelected();
		switch(_autonModeChoice) {
		case CROSS_BASE_LINE:
			return new CrossBaseLine();
		case DO_NOTHING:
			return new DoNothing();
		case HANG_BOILER_GEAR:
			return new HangBoilerGear(getIsBlueAlliance());
		case HANG_BOILER_GEAR_AND_SHOOT:
			return new HangBoilerGearAndShoot(getIsBlueAlliance());
		case HANG_CENTER_GEAR:
			return new CenterGear();
		case HANG_CENTER_GEAR_AND_SHOOT:
			return new CenterGearAndShoot();
		case HANG_RETRIEVAL_GEAR:
			return new HangRetrievalGear(getIsBlueAlliance());
		case HIT_HOPPER:
			return new HitHopper();
		case TURN_AND_SHOOT:
			return new TurnAndShoot();
		case TWO_GEAR:
			return new TwoGear();
		case UNDEFINED:
			return new DoNothing();
		default:
			return new DoNothing();
		}
	}
	
	public void printStartupMessage() {
		// This prints once during robotInit
		boolean isFMSAttached = getIsFMSAttached();
		ALLIANCE_COLOR allianceColor = _allianceChooser.getSelected();
		_fmsDebugMsg = "Is FMS Attached: [" + isFMSAttached + "] Alliance: [" + allianceColor + "]";
		DriverStation.reportWarning(">>>>> " + _fmsDebugMsg + " <<<<<<", false);
	}
	
	public boolean getIsBlueAlliance() {
		_allianceColor = _allianceChooser.getSelected();
		
		switch (_allianceColor) {
		case BLUE_ALLIANCE:
			return true;
			
		case RED_ALLIANCE:
			return false;
			
		case USE_FMS:
			if(getIsFMSAttached()) {
				Alliance fmsAlliance = DriverStation.getInstance().getAlliance();
				
				switch(fmsAlliance) {
					case Blue:
						return true;
						
					case Red:
						return false;
						
					default:
						return true;	// force default of blue alliance
				}
			} else {
				return true;	// force default of blue alliance
			}
		
		default:
			return true;
		}
	}
	
	public CAMERA_NAMES get_gearCam() {
		_gearCameraName = _gearCamChooser.getSelected();
		return _gearCameraName;
	}
	
	public CAMERA_NAMES get_shooterCam() {
		_shooterCameraName = _shooterCamChooser.getSelected();
		return _shooterCameraName;
	}
	
	public CAMERA_NAMES get_climberCam() {
		_climberCameraName = _climberCamChooser.getSelected();
		return _climberCameraName;
	}
	
	public CAMERA_NAMES get_driverCam() {
		_driverCameraName = _driverCamChooser.getSelected();
		return _driverCameraName;
	}
}