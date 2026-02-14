// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Drive.GyroPigeon2;
import frc.robot.Subsystems.Drive.GyroSim;
import frc.robot.Subsystems.Drive.ModuleSim;
import frc.robot.Subsystems.Drive.ModuleTalonFX;
import frc.robot.Subsystems.Vision.CameraIO;
import frc.robot.Subsystems.Vision.CameraIOPhotonVision;
import frc.robot.Subsystems.Vision.Vision;
import frc.robot.Subsystems.Vision.VisionConstants;
import frc.robot.Subsystems.Vision.VisionConstants.Orientation;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    public final CommandXboxController kDriveController = new CommandXboxController(RobotConstants.getInstance().kDriveControllerPort);
    public Drive kDrive;

  	public RobotContainer() {
    	switch (RobotConstants.getInstance().kMode) {
            case REAL:
                kDrive = new Drive(
                    new ModuleTalonFX(RobotConstants.DriveConstants().kFRModuleIDs, RobotConstants.DriveConstants().kFRModuleOffsets, RobotConstants.DriveConstants().kModuleGains, "drivebase"),
                    new ModuleTalonFX(RobotConstants.DriveConstants().kFLModuleIDs, RobotConstants.DriveConstants().kFLModuleOffsets, RobotConstants.DriveConstants().kModuleGains, "drivebase"),
                    new ModuleTalonFX(RobotConstants.DriveConstants().kBRModuleIDs, RobotConstants.DriveConstants().kBRModuleOffsets, RobotConstants.DriveConstants().kModuleGains, "drivebase"),
                    new ModuleTalonFX(RobotConstants.DriveConstants().kBLModuleIDs, RobotConstants.DriveConstants().kBLModuleOffsets, RobotConstants.DriveConstants().kModuleGains, "drivebase"),
                    new GyroPigeon2(RobotConstants.DriveConstants().kGyroID, RobotConstants.DriveConstants().kGyroOffsets, "drivebase"),
					new Vision(new CameraIO[] {
						new CameraIOPhotonVision(VisionConstants.kRightCamName, VisionConstants.kRightCamTransform, Orientation.FRONT), 
                    	new CameraIOPhotonVision(VisionConstants.kLeftCamName, VisionConstants.kLeftCamTransform, Orientation.FRONT)
					})
                );
                break;
            case REPLAY:
                break;
            case SIM:
                kDrive = new Drive(
                    new ModuleSim(), 
                    new ModuleSim(), 
                    new ModuleSim(), 
                    new ModuleSim(), 
                    new GyroSim(),
					new Vision(new CameraIO[] {
						new CameraIOPhotonVision(VisionConstants.kRightCamName, VisionConstants.kRightCamTransform, Orientation.FRONT), 
                    	new CameraIOPhotonVision(VisionConstants.kLeftCamName, VisionConstants.kLeftCamTransform, Orientation.FRONT)
					})
                );
                break;
            default:
                break;
        }

    	configureBindings();
  	}

  	private void configureBindings() {
        DriverStation.silenceJoystickConnectionWarning(true);

        kDrive.setDefaultCommand(new InstantCommand(() -> kDrive.setDriveState(Drive.driveState.TELEOP), kDrive));

        kDrive.supplyControllerInputs(() -> kDriveController.getLeftX(), () -> kDriveController.getLeftY(), () -> kDriveController.getRightX());
    
        kDriveController.a().debounce(0.25d, DebounceType.kRising)
            .onTrue(new InstantCommand(() -> kDrive.setDriveState(Drive.driveState.SYSID)).andThen(kDrive.getSysIdCommand()))
            .onFalse(kDrive.getDefaultCommand());
    }
}