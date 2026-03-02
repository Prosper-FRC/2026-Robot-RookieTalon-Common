package frc.robot.SubSystems.Intake;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private final IntakeIO rollerIO;
  private final PivotIO pivotIO;

private final IntakeIO.IntakeInputs rollerInputs =
    new IntakeIO.IntakeInputs();

private final PivotIO.PivotIOInputs pivotInputs =
    new PivotIO.PivotIOInputs();

  public Intake(IntakeIO rollerIO, PivotIO pivotIO) {
    this.rollerIO = rollerIO;
    this.pivotIO = pivotIO;
  }

  @Override
  public void periodic() {
    rollerIO.updateInputs(rollerInputs);
    pivotIO.updateInputs(pivotInputs);

    Logger.recordOutput("Intake/Roller/Velocity", rollerInputs.velocityRPS);
    Logger.recordOutput("Intake/Pivot/Angle",
    pivotInputs.position.getDegrees());
  }

  /* ================= ROLLER ================= */

  public void intake() {
    rollerIO.setVoltage(10);
  }

  public void outtake() {
    rollerIO.setVoltage(0);
  }

  public void stopRoller() {
    rollerIO.stop();
  }

  /* ================= PIVOT ================= */

  public void movePivotTo(Rotation2d position) {
    pivotIO.setPosition(position);
  }

  public void stopPivot() {
    pivotIO.stop();
  }

  public boolean atPivotPosition(Rotation2d target) {
    return Math.abs(
        pivotInputs.position.minus(target)
            .getDegrees())
        < IntakeConstants.kPivotTolerance
            .getDegrees();
  }
}
