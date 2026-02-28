package frc.robot.Subsystems.Shooter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotConstants;

public class ShooterConstants{
    public static ShooterConstants instance = null;
    public final int kTeamNumber = RobotConstants.getInstance().kTeamNumber;

    //Flywheel
    public int kFlywheelmotorId;

    //Hood
    public int kHoodMotorId;
    public int kHoodCancoderId;
    public static double kHooderArmLengthMeters = 0.15d;
    public static Rotation2d kHooderMaxAngleRads = new Rotation2d(Units.degreesToRadians(360.0));

    //Indexer
    public int kIndexerMotorId;
    public static double kIndexerMaxVelocityRPM = 10.0d;
    public static double kIndexerMaxAccelerationRPM = 5.0d;

    //NEED TO CALCULATE
    public static final Rotation2d kHoodPosition1 = Rotation2d.fromRotations(0.0); //y value at ??
    public static final Rotation2d kHoodPosition2 = Rotation2d.fromRotations(0.0); //y value at ??
    public static final Rotation2d kHoodPosition3 = Rotation2d.fromRotations(0.0); //y value at ??

    private ShooterConstants() {
        switch(kTeamNumber) {
            case 9999:
                //motors for bot
            case 0:
                kFlywheelmotorId = 1;
                kHoodMotorId = 2;
                kHoodCancoderId = 3;
                kIndexerMotorId = 4;
                break;
        }
    }

    public static ShooterConstants getInstance() {
        if (instance == null) {
            instance = new ShooterConstants();

        }

        return instance;
    }

}