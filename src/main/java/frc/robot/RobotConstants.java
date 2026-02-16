package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;

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
                break;
            case 0:
                break;
            default:
                break;
        }
    }

    public static RobotConstants getInstance() {
        if (instance == null) {
            instance = new RobotConstants();
        }
        return instance;
    }
}