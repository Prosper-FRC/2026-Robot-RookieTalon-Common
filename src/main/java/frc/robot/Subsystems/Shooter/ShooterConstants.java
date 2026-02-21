package frc.robot.Subsystems.Shooter;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotConstants;

public class ShooterConstants{
    public static ShooterConstants instance = null;
    public final int kTeamNumber = RobotConstants.getInstance().kTeamNumber;

    //Flywheel ID
    public int kFlywheelmotorId;

    //Hood ID
    public int kHoodMotorId;
    public int kHoodCancoderId;

    //Indexer ID
    public int kIndexerMotorId;

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