package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.math.geometry.Rotation2d;

import frc.robot.SubSystems.Intake.*;

public class RobotContainer {

  /* ================= CONTROLLER ================= */

  private final CommandXboxController driver =
      new CommandXboxController(0);

  /* ================= SUBSYSTEMS ================= */

  private final Intake intake;

  public RobotContainer() {

    /* ----------- IO SELECTION (REAL vs SIM) ----------- */

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

  /* ================= BUTTON BINDINGS ================= */

  private void configureBindings() {

    /* ---------- ROLLER ---------- */

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

    /* ---------- MANUAL PIVOT CONTROL ---------- */

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

  /* ================= OPTIONAL HELPER ================= */

  private double intakePivotAngleDegrees() {
    // Safe placeholder if you want manual adjustments
    return 0;
  }

  /* ================= AUTONOMOUS ================= */

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