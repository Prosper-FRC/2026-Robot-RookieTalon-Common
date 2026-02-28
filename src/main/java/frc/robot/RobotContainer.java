// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Shooter.ShooterIOSim;
import frc.robot.Subsystems.Shooter.ShooterConstants;

public class RobotContainer {
  public final CommandXboxController kDriveController = new CommandXboxController(RobotConstants.getInstance().kDriveControllerPort);
  public Shooter kShooter;

  public RobotContainer() {
    	switch (RobotConstants.getInstance().kMode) {
          case REAL:
                //STUFF
                break;
            case REPLAY:
                break;
            case SIM:
                kShooter = new Shooter(new ShooterIOSim());
                break;
            default:
                break;
        }

    	configureBindings();
  	}

	private void configureBindings() {

        DriverStation.silenceJoystickConnectionWarning(true);

        //bind one button for shooting, one button for hood position

		
        kDriveController.rightBumper().whileTrue(
            Commands.run(() -> kShooter.setFlywheelVelocity(3000), kShooter)
        ).onFalse(
            Commands.runOnce(kShooter::stopFlywheel, kShooter)
        );

        
        kDriveController.a().onTrue(
            Commands.runOnce(() -> kShooter.setHooderPositionRotations(ShooterConstants.kHoodPosition1), kShooter)
        );

        kDriveController.b().onTrue(
            Commands.runOnce(() -> kShooter.setHooderPositionRotations(ShooterConstants.kHoodPosition2), kShooter)
        );

        kDriveController.x().onTrue(
            Commands.runOnce(() -> kShooter.setHooderPositionRotations(ShooterConstants.kHoodPosition3), kShooter)
        );
     
        kDriveController.start().onTrue(
            Commands.runOnce(() -> {
                kShooter.stopFlywheel();
                kShooter.stopHooder();
            }, kShooter)
        );
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

}