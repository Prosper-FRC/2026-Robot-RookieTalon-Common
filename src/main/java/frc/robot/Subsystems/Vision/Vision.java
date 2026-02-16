package frc.robot.Subsystems.Vision;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import frc.robot.utils.debugging.LoggedTunableNumber;
import static frc.robot.Subsystems.Vision.VisionConstants.KUseSingleTagTransform;
import static frc.robot.Subsystems.Vision.VisionConstants.kSingleStdDevs;
import static frc.robot.Subsystems.Vision.VisionConstants.kMultiStdDevs;
import static frc.robot.Subsystems.Vision.VisionConstants.kAmbiguityThreshold;;

public class Vision {
    private CameraIO[] cameras;
    private CameraIOInputsAutoLogged[] camerasData;

    private static final LoggedTunableNumber kSingleStdDev = new LoggedTunableNumber("Vision/kSingleStdDev", kSingleStdDevs.get(0));
    private static final LoggedTunableNumber kMultiStdDev = new LoggedTunableNumber("Vision/kMultiStdDev", kMultiStdDevs.get(0));

    private final AprilTagFieldLayout k2026Field = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    public Vision(CameraIO[] cameras) {
        this.cameras = cameras;
        camerasData = new CameraIOInputsAutoLogged[cameras.length];
        for (int i = 0; i < cameras.length; i++) {
            camerasData[i] = new CameraIOInputsAutoLogged();
        }
        Logger.recordOutput("Vision/UseSingleTagTransform", KUseSingleTagTransform);
    }

    public void periodic(Pose2d lastRobotPose, Pose2d simOdmPose) {
        for (int i = 0; i < cameras.length; i++) {
            cameras[i].updateInputs(camerasData[i], lastRobotPose, simOdmPose);
            Logger.processInputs("Vision/" + camerasData[i].camName, camerasData[i]);
        }
    }

    public VisionObservation[] getVisionObservations() {
        VisionObservation[] observations = new VisionObservation[cameras.length];
        int i = 0;

        for (CameraIOInputsAutoLogged cameraData : camerasData) {
            if (cameraData.hasBeenUpdated && cameraData.hasTarget) {
                int numberOfTargets = cameraData.numberOfTargets;
                double totalDistanceMeters = 0.0;

                for (int j = 0; j < cameraData.latestTagTransforms.length; j++) {
                    if (cameraData.latestTagTransforms[j] != null) {
                        if (cameraData.latestTagAmbiguities[j] < kAmbiguityThreshold) {
                            totalDistanceMeters += cameraData.latestTagTransforms[j].getTranslation().getNorm();
                        } else {
                            numberOfTargets--;
                        }
                    }
                }

                if (numberOfTargets == 0) {
                    observations[i] = new VisionObservation(
                        true, 
                        cameraData.latestEstimatedRobotPose.toPose2d(), 
                        VecBuilder.fill(
                            Double.MAX_VALUE,
                            Double.MAX_VALUE,
                            Double.MAX_VALUE
                        ), 
                        cameraData.latestTimestamp, 
                        cameraData.camName
                    );

                    i++;
                    continue;
                }

                double averageDistanceMeters = totalDistanceMeters / numberOfTargets;
                double xyScalar = Math.pow(averageDistanceMeters, 2) / (numberOfTargets);

                if ((numberOfTargets == 1) && (averageDistanceMeters < 3.5)) {
                    observations[i] = new VisionObservation(
                        true, 
                        cameraData.latestEstimatedRobotPose.toPose2d(), 
                        VecBuilder.fill(
                            Double.MAX_VALUE,
                            Double.MAX_VALUE,
                            Double.MAX_VALUE
                        ), 
                        cameraData.latestTimestamp, 
                        cameraData.camName
                    );
                } else if (numberOfTargets == 1) {
                    Pose2d singleTagPose = new Pose2d();

                    if(KUseSingleTagTransform) {
                        singleTagPose = 
                            k2026Field.getTagPose(cameraData.singleTagAprilTagID).get().toPose2d()
                            .plus(new Transform2d(
                                    cameraData.cameraToApriltag.getX(), cameraData.cameraToApriltag.getY(), 
                                    cameraData.cameraToApriltag.getRotation().toRotation2d()))
                            .plus(toTransform2d(cameraData.cameraToRobot.inverse()));
                    } else {
                        singleTagPose = cameraData.latestEstimatedRobotPose.toPose2d();
                    }
                    observations[i] = new VisionObservation(
                        true,
                        singleTagPose, 
                        VecBuilder.fill(
                            kSingleStdDev.get() * xyScalar, 
                            kSingleStdDev.get() * xyScalar, 
                            Double.MAX_VALUE), 
                        cameraData.latestTimestamp, cameraData.camName
                    );
                } else {
                    observations[i] = new VisionObservation(
                        true,
                        cameraData.latestEstimatedRobotPose.toPose2d(), 
                        VecBuilder.fill(
                            kMultiStdDev.get() * xyScalar, 
                            kMultiStdDev.get() * xyScalar, 
                            Double.MAX_VALUE), 
                        cameraData.latestTimestamp, cameraData.camName
                    );
                }
            } else {
                observations[i] = new VisionObservation(
                    false, 
                    new Pose2d(), 
                    VecBuilder.fill(
                        Double.MAX_VALUE,
                        Double.MAX_VALUE,
                        Double.MAX_VALUE
                    ), 
                    cameraData.latestTimestamp, 
                    cameraData.camName
                );
            }
            i++;
        }
        
        return observations;
    }

    public Transform2d toTransform2d(Transform3d transform3d) {
        return new Transform2d(transform3d.getX(), transform3d.getY(), transform3d.getRotation().toRotation2d());
    }

    public void logVisionObservation(VisionObservation observation, String state) {
        Logger.recordOutput("Vision/Observation/" + observation.camName + "/State", state);
        Logger.recordOutput("Vision/Observation/" + observation.camName + "/Timestamp", observation.camName());
        Logger.recordOutput("Vision/Observation/" + observation.camName + "/Pose", observation.pose());
        Logger.recordOutput("Vision/Observation/" + observation.camName + "/hasObserved", observation.hasObserved());
        Logger.recordOutput("Vision/Observation/" + observation.camName + "/StdDevs", observation.stdDevs());
    }

    public record VisionObservation(boolean hasObserved, Pose2d pose, Vector<N3> stdDevs, double timeStamp, String camName) {} 
}