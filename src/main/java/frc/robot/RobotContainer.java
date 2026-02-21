// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.math.geometry.Rotation2d;

import frc.robot.Subsystems.Intake.*;

public class RobotContainer {



  private final CommandXboxController driver =
      new CommandXboxController(0);


  private final Intake intake;

  public RobotContainer() {


    IntakeIO rollerIO;
    PivotIO pivotIO;

    if (RobotBase.isReal()) {
      rollerIO = new IntakeIOTalonFX();
      pivotIO = new PivotIOTalonFX();
    } else {
      rollerIO = new IntakeIOTalonFX(); // roller sim optional
      pivotIO = new PivotIOSim();
    }

    intake = new Intake(rollerIO, pivotIO);

    configureBindings();
  }



  private void configureBindings() {

    // Right trigger = intake
    driver.rightTrigger()
        .whileTrue(
            Commands.run(
                () -> intake.intake(),
                intake))
        .onFalse(
            Commands.runOnce(
                () -> intake.stopRoller(),
                intake));

    // Left trigger = outtake
    driver.leftTrigger()
        .whileTrue(
            Commands.run(
                () -> intake.outtake(),
                intake))
        .onFalse(
            Commands.runOnce(
                () -> intake.stopRoller(),
                intake));

    /* ---------- PIVOT POSITIONS ---------- */

    // A button = stowed
    driver.a()
        .onTrue(
            Commands.runOnce(
                () -> intake.movePivotTo(
                    IntakeConstants.kMinPivotPosition),
                intake));

    // Y button = deployed
    driver.y()
        .onTrue(
            Commands.runOnce(
                () -> intake.movePivotTo(
                    IntakeConstants.kMaxPivotPosition),
                intake));



    // Left joystick Y for manual voltage control
    intake.setDefaultCommand(
        Commands.run(() -> {

          double stick = -driver.getLeftY();

          if (Math.abs(stick) > 0.1) {
            intake.movePivotTo(
                Rotation2d.fromDegrees(
                    intakePivotAngleDegrees() + stick * 2));
          }

        }, intake));
  }


  private double intakePivotAngleDegrees() {
    // Safe placeholder if you want manual adjustments
    return 0;
  }

  public Command getAutonomousCommand() {

    return Commands.sequence(

        Commands.runOnce(
            () -> intake.movePivotTo(
                IntakeConstants.kMaxPivotPosition),
            intake),

        Commands.waitSeconds(1.0),

        Commands.runOnce(
            () -> intake.intake(),
            intake),

        Commands.waitSeconds(2.0),

        Commands.runOnce(
            () -> intake.stopRoller(),
            intake),

        Commands.runOnce(
            () -> intake.movePivotTo(
                IntakeConstants.kMinPivotPosition),
            intake)
    );
  }
}