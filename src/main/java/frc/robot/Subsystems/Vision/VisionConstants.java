package frc.robot.Subsystems.Vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotBase;

public class VisionConstants {
    public static final String kLeftCamName = "5411_LEFT";
    public static final Orientation kLeftCamOrientation = Orientation.BACK;
    public static final Transform3d kLeftCamTransform = new Transform3d(
        new Translation3d(0.0, 0.0, 0.0),
        new Rotation3d(0.0, Math.toRadians(8.317), Math.toRadians(-25.0))
    );
  
    public static final String kRightCamName = "5411_RIGHT";
    public static final Orientation kRightCamOrientation = Orientation.BACK;
    public static final Transform3d kRightCamTransform = new Transform3d(
        new Translation3d(0.0, 0.0, 0.0),
        new Rotation3d(0.0, Math.toRadians(8.317), Math.toRadians(25.0))
    );

    public static final boolean KUseSingleTagTransform = false;

    // RETUNE FOR THIS YEAR
    // Tuned by using AdvantageScope data analysis tool(Normal distribution) LAST YEAR
    /* https://docs.google.com/document/d/16ryTjwguRXpwBKdGc8rs4iRQL18iNsfMrIzfY3cchzU/edit?usp=sharing */
    public static final Vector<N3> kSingleStdDevs = (RobotBase.isReal()) ?
        VecBuilder.fill(0.274375, 0.274375, 5.0) : VecBuilder.fill(0.01, 0.01, 5.0);
    public static final Vector<N3> kMultiStdDevs = (RobotBase.isReal()) ?
        VecBuilder.fill(0.274375, 0.274375, 5.0) : VecBuilder.fill(0.01, 0.01, 5.0);

    public static final double kAmbiguityThreshold = (RobotBase.isReal()) ? 0.2 : 1.0;

    public static final Rotation2d kOV2311DiagonalCameraFOV = Rotation2d.fromDegrees(95.0);

    public static enum Orientation {
        BACK,
        FRONT
    }
}
