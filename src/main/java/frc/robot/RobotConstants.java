package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Drive.DriveConstants.DriveConstants;
import frc.robot.Subsystems.Drive.DriveConstants.DriveConstants9999;
import frc.robot.Subsystems.Drive.DriveConstants.DriveConstantsSim;

public class RobotConstants {
    private static RobotConstants instance = null;

    // MAKE THIS FALSE BEFORE SCRIMMAGES
    public static final boolean kTuningMode = true;

    public static enum mode {
        REAL,
        REPLAY,
        SIM
    };

    public final int kTeamNumber;
    public final mode kMode;
    public final int kDriveControllerPort = 0;
    public final double kTimestep = 0.02d;

    private final DriveConstants kDriveConstants;

    private RobotConstants() {
        kTeamNumber = RobotController.getTeamNumber();
        if(RobotBase.isReal()) {
            kMode = mode.REAL;
        } else if(RobotBase.isSimulation()) {
            kMode = mode.SIM;
        } else {
            kMode = mode.REPLAY;
        }

        switch (kTeamNumber) {
            case 9999:
                kDriveConstants = new DriveConstants9999();
                break;
            case 0:
                kDriveConstants = new DriveConstantsSim();
                break;
            default:
                kDriveConstants = new DriveConstants();
                break;
        }
    }

    public static DriveConstants DriveConstants() {
        return instance.kDriveConstants;
    }

    public static RobotConstants getInstance() {
        if (instance == null) {
            instance = new RobotConstants();
        }
        return instance;
    }
}