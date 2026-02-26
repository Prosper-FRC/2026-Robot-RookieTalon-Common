package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Intake.IntakeIOTalonFX;
import frc.robot.Subsystems.Intake.IntakeIOSim;
import frc.robot.Subsystems.Intake.PivotIOTalonFX;
import frc.robot.Subsystems.Intake.PivotIOSim;

public class RobotContainer {
  private final CommandXboxController driver = new CommandXboxController(0);

  // ✅ Correct constructor: Intake(IntakeIO, PivotIO)
  private final Intake intake =
      new Intake(
          RobotBase.isReal() ? new IntakeIOTalonFX() : new IntakeIOSim(),
          RobotBase.isReal() ? new PivotIOTalonFX() : new PivotIOSim()
      );

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // ⚠️ These methods must exist in Intake (see section 2 below)
    driver.rightBumper()
        .whileTrue(new InstantCommand(() -> intake.setIntakePercent(0.8), intake))
        .onFalse(new InstantCommand(() -> intake.setIntakePercent(0.0), intake));

    driver.leftBumper()
        .whileTrue(new InstantCommand(() -> intake.setIntakePercent(-0.8), intake))
        .onFalse(new InstantCommand(() -> intake.setIntakePercent(0.0), intake));

    driver.a().onTrue(new InstantCommand(() -> intake.setIntakePercent(0.0), intake));
  }

  public Command getAutonomousCommand() {
    return new InstantCommand(() -> DriverStation.reportWarning("No auto selected", false));
  }
}