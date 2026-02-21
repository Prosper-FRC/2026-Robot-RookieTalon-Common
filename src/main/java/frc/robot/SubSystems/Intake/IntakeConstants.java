package frc.robot.Subsystems.Intake;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public final class IntakeConstants {

  // ================= ROLLER =================

  public static final int kRollerMotorId = 56;

  public static final double kRollerGearing = 3.0;
  public static final double kWheelRadiusMeters = Units.inchesToMeters(0.5);

  // ================= PIVOT =================

  public static final int kPivotMotorId = 58;

  public static final double kPivotGearing =
      (48.0 / 1.0) * (48.0 / 22.0);

  public static final Rotation2d kMinPivotPosition =
      Rotation2d.fromDegrees(-70.0);

  public static final Rotation2d kMaxPivotPosition =
      Rotation2d.fromDegrees(64.0);

  public static final Rotation2d kPivotTolerance =
      Rotation2d.fromDegrees(1.0);

  // ================= PIVOT GAINS =================

  public static final double kPivotP = 0.0;
  public static final double kPivotI = 0.0;
  public static final double kPivotD = 0.0;

  public static final double kPivotS = 0.0;
  public static final double kPivotV = 13.0;
  public static final double kPivotA = 0.0;
  public static final double kPivotG = 0.0;

  public static final double kCruiseVelocity = 1000;
  public static final double kAcceleration = 500;
  public static final double kJerk = 0;

  public static final NeutralModeValue kPivotNeutral =
      NeutralModeValue.Brake;
}
