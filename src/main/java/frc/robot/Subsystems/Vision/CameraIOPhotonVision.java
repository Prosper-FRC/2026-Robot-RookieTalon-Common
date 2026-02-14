package frc.robot.Subsystems.Vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.mode;
import frc.robot.Subsystems.Vision.VisionConstants.Orientation;

public class CameraIOPhotonVision implements CameraIO {
    private String cameraName;
    private PhotonCamera photonCamera;
    private PhotonPoseEstimator poseEstimator;
    private PhotonCameraSim photonCameraSim;
    private VisionSystemSim visionSystemSim;
    private PhotonPoseEstimator photonPoseEstimator;
    private Transform3d cameraTransform;
    private Orientation orientation;

    public CameraIOPhotonVision(String cameraName, Transform3d cameraTransform, Orientation orientation) {
        this.cameraName = cameraName;
        this.cameraTransform = cameraTransform;
        this.orientation = orientation;

        this.photonCamera = new PhotonCamera(cameraName);
        PhotonCamera.setVersionCheckEnabled(false);

        poseEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark), cameraTransform);
        //

        if (RobotConstants.getInstance().kMode == mode.SIM) {
            visionSystemSim = new VisionSystemSim("main");
            visionSystemSim.addAprilTags(AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark));

            SimCameraProperties cameraProp = new SimCameraProperties();
            cameraProp.setCalibError(0.3, 0.20);
            cameraProp.setFPS(60);
            cameraProp.setAvgLatencyMs(5);
            cameraProp.setLatencyStdDevMs(15);

            photonCameraSim = new PhotonCameraSim(photonCamera);

            visionSystemSim.addCamera(photonCameraSim, cameraTransform);
        }
    }

    @Override
    public void updateInputs(CameraIOInputs inputs, Pose2d lastRobotPose, Pose2d simOdmPose) {
        inputs.camName = cameraName;
        inputs.cameraToRobot = cameraTransform;

        try {
            if (RobotConstants.getInstance().kMode == mode.SIM) {
                visionSystemSim.update(simOdmPose);
                visionSystemSim.getDebugField();
            }

            List<PhotonPipelineResult> results = photonCamera.getAllUnreadResults();

            inputs.hasBeenUpdated = !results.isEmpty();
            if (!results.isEmpty()) {
                PhotonPipelineResult result = results.get(results.size()-1);

                inputs.isConnected = photonCamera.isConnected();
                inputs.hasTarget = result.hasTargets();

                if (result.hasTargets()) {
                    Optional<EstimatedRobotPose> visionEstimatedPose;
                    Optional<EstimatedRobotPose> multiTagEstiamtedPose = poseEstimator.estimateCoprocMultiTagPose(result);
                    visionEstimatedPose = multiTagEstiamtedPose.isEmpty()? 
                        poseEstimator.estimateClosestToReferencePose(result, new Pose3d(lastRobotPose))
                        : multiTagEstiamtedPose;

                    PhotonTrackedTarget target = result.getBestTarget();
                    inputs.cameraToApriltag = target.getBestCameraToTarget();
                    inputs.robotToApriltag = target.getBestCameraToTarget().plus(cameraTransform);
                    inputs.singleTagAprilTagID = target.getFiducialId();
                    inputs.poseAmbiguity = target.getPoseAmbiguity();
                    inputs.yaw = target.getYaw();
                    inputs.pitch = target.getPitch();
                    inputs.area = target.getArea();
                    inputs.latencySeconds = result.getTimestampSeconds() / 1000.0;

                    visionEstimatedPose.ifPresent(est -> {
                        if (orientation.equals(Orientation.FRONT)){
                            inputs.latestEstimatedRobotPose = visionEstimatedPose.get().estimatedPose;
                        } else {
                            inputs.latestEstimatedRobotPose = visionEstimatedPose.get().estimatedPose
                                .transformBy(new Transform3d(
                                    new Translation3d(), new Rotation3d(0.0, 0.0, Math.PI)));
                        }

                        ArrayList<Transform3d> tagTransforms = new ArrayList<>();
                        double[] ambiguities = new double[visionEstimatedPose.get().targetsUsed.size()];

                        if (visionEstimatedPose.get().targetsUsed.size() > 0) {
                            for(int i = 0; i < visionEstimatedPose.get().targetsUsed.size(); i++) {
                                tagTransforms.add(visionEstimatedPose.get().targetsUsed.get(i).getBestCameraToTarget());
                                ambiguities[i] = visionEstimatedPose.get().targetsUsed.get(i).getPoseAmbiguity();
                            }
                        }

                        inputs.numberOfTargets = visionEstimatedPose.get().targetsUsed.size();
                        inputs.latestTagTransforms = tagTransforms.toArray(Transform3d[]::new);
                        inputs.latestTagAmbiguities = ambiguities;
        
                        inputs.latestTimestamp = result.getTimestampSeconds();
                    });
                }
            }

        } catch (Exception e) {
            e.printStackTrace();
            inputs.isConnected = false;
            inputs.yaw = 0.0;
            inputs.pitch = 0.0;
            inputs.area = 0.0;
            inputs.latencySeconds = 0.0;
            inputs.hasTarget = false;
            inputs.numberOfTargets = 0;
        
            inputs.cameraToApriltag = new Transform3d();
            inputs.poseAmbiguity = 0.0;
            inputs.singleTagAprilTagID = 0;
            inputs.robotToApriltag = new Transform3d();
            inputs.latestTimestamp = 0.0;
            inputs.latestEstimatedRobotPose = new Pose3d();
            inputs.latestTagTransforms = new Transform3d[14];
            inputs.latestTagAmbiguities = new double[14];
        }
    }
}
