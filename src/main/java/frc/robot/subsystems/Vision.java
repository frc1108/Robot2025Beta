package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.BiConsumer;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.TagVisionConstants;

@Logged
public class Vision extends SubsystemBase {
    private final PhotonCamera photonCamera;    
    private final PhotonPoseEstimator poseEstimator;
    private final AprilTagFieldLayout fieldLayout;
    private final BiConsumer<Pose2d, Double> consumer;
    private final DriveSubsystem drive;

    public Vision(BiConsumer<Pose2d, Double> consumer, DriveSubsystem drive) throws IOException{
        photonCamera = new PhotonCamera(Constants.TagVisionConstants.kCameraName);
        fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        poseEstimator = new PhotonPoseEstimator(
            fieldLayout,
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            Constants.TagVisionConstants.kCameraOffset);
        this.consumer = consumer;
        this.drive = drive;
    }

    @Override
    public void periodic(){
      boolean connected = photonCamera.isConnected();
      if (!connected)
          return;

        PhotonPipelineResult pipelineResult = photonCamera.getAllUnreadResults().get(0);  //TODO check if this is right change
        boolean hasTargets = pipelineResult.hasTargets();
        if (!hasTargets)
            return;

        List<PhotonTrackedTarget> badTargets = new ArrayList<>();
        for(PhotonTrackedTarget target : pipelineResult.targets){
            var tagPose = fieldLayout.getTagPose(target.getFiducialId());
            var distanceToTag = PhotonUtils.getDistanceToPose(drive.getPose(),tagPose.get().toPose2d());
            if(target.getPoseAmbiguity()>0.35 || distanceToTag > TagVisionConstants.kMaxDistanceMeters){  // Changed from 0.5 ambiguity
                badTargets.add(target);
            }
        }

        pipelineResult.targets.removeAll(badTargets);
                
        Optional<EstimatedRobotPose> poseResult = poseEstimator.update(pipelineResult);
        boolean posePresent = poseResult.isPresent();
        if (!posePresent)
            return;

        EstimatedRobotPose estimatedPose = poseResult.get();

        consumer.accept(estimatedPose.estimatedPose.toPose2d(), estimatedPose.timestampSeconds);
    }


}