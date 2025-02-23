package frc.robot.subsystems;
import static edu.wpi.first.units.Units.Meter;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class PhotonVision extends SubsystemBase {
    private CommandSwerveDrivetrain dt;
    private PhotonCamera camera;
    private PhotonPoseEstimator photonPoseEstimator;
    EstimatedRobotPose lastPose = null;
    private PoseStrategy strat;
    private Transform3d mRobotToCam;
    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
    public PhotonVision(CommandSwerveDrivetrain drive, String cameraName, PoseStrategy realStrat, Transform3d rbtc){
        dt = drive;
        camera = new PhotonCamera(cameraName);
        strat = realStrat;
        mRobotToCam = rbtc;
        // Construct PhotonPoseEstimator
        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, strat, mRobotToCam);
    }

    public PhotonCamera getCamera() {
        return camera;

    }

    public void resetPose(){
        if (camera.isConnected()) {
            Optional<EstimatedRobotPose> pose = getEstimatedGlobalPose(dt.getState().Pose);
            PhotonPipelineResult latestResult = camera.getLatestResult();
            if(latestResult.hasTargets()){
                PhotonTrackedTarget trackedTarget = latestResult.getBestTarget();
                
                
                if(pose.isPresent() && trackedTarget != null
                && trackedTarget.getBestCameraToTarget() != null
                && ((trackedTarget.getBestCameraToTarget().getTranslation().getX() < 1.25 && (DriverStation.isDisabled() || DriverStation.isAutonomous())) 
                || (trackedTarget.getBestCameraToTarget().getTranslation().getX() < 1.25 && DriverStation.isTeleop())
                )) {
                    if (!rejectPose()) {
                        double stdev = 0.01 * trackedTarget.getBestCameraToTarget().getTranslation().getX();
                        dt.setVisionMeasurementStdDevs(VecBuilder.fill(stdev, stdev, 0.5));
                        dt.addVisionMeasurement(pose.get().estimatedPose.toPose2d(), pose.get().timestampSeconds);
                        lastPose = pose.get();
                    }
                }
            }
        } 
    }

    
    public void resetPoseFixed(){
        if (camera.isConnected()) {
            Optional<EstimatedRobotPose> pose = getEstimatedGlobalPose(dt.getState().Pose);
            var results = camera.getAllUnreadResults();
            if(!results.isEmpty()){
                var latestResult = results.get(results.size() - 1);
                if (latestResult.hasTargets()) {
                    PhotonTrackedTarget trackedTarget = latestResult.getBestTarget();
                    if(pose.isPresent() && trackedTarget != null
                    && trackedTarget.getBestCameraToTarget() != null
                    && ((trackedTarget.getBestCameraToTarget().getTranslation().getX() < 3.5 && (DriverStation.isDisabled() || DriverStation.isAutonomous())) 
                    || (trackedTarget.getBestCameraToTarget().getTranslation().getX() < 6 && DriverStation.isTeleop())
                    )) {
                        if (!rejectPose()) {
                            if (pose != null) {
                                double stdev = 0.01 * trackedTarget.getBestCameraToTarget().getTranslation().getX();
                                dt.setVisionMeasurementStdDevs(VecBuilder.fill(stdev, stdev, 0.5));
                                dt.addVisionMeasurement(pose.get().estimatedPose.toPose2d(), pose.get().timestampSeconds);
                                lastPose = pose.get();
                            }
                        }
                    }
                }
            }
        }
    }

    public boolean rejectPose() {
        if (dt.getState().Speeds.omegaRadiansPerSecond > 360) {
            return true;
        } else {
            return false;
        }
    }


    @Override
    public void periodic(){
      resetPose();
    }

    public double getRobotHeading(){
        if (camera.isConnected()) {
            double heading = 0;
            if(lastPose != null){
                heading = lastPose.estimatedPose.getRotation().toRotation2d().getDegrees();
            }
            var plresults = camera.getAllUnreadResults();
            PhotonPipelineResult plresult = plresults.get(plresults.size() - 1);
            if(plresult.getBestTarget() != null && plresult.hasTargets()){
                PhotonTrackedTarget trackedTarget = plresult.getBestTarget();
                if(trackedTarget != null && trackedTarget.getBestCameraToTarget() != null && aprilTagFieldLayout.getTagPose(trackedTarget.getFiducialId()).isPresent()){
                    heading = (aprilTagFieldLayout.getTagPose(trackedTarget.getFiducialId()).get().getRotation().toRotation2d().getDegrees() - (trackedTarget.getBestCameraToTarget().getRotation().toRotation2d().getDegrees()));
                }
            }
            return heading;
        } else {
            return 0;
        }
    }



    public double getTagXDist() {
        double tagX = 0;
        if (camera.isConnected()) {
            List<PhotonPipelineResult> currentUnreadPipeline = camera.getAllUnreadResults();
            if (!currentUnreadPipeline.isEmpty()) {
                PhotonPipelineResult latestTag = currentUnreadPipeline.get(currentUnreadPipeline.size() - 1);
                if (latestTag.hasTargets() || latestTag.getBestTarget() != null) {
                    Transform3d camToTarget = latestTag.getBestTarget().bestCameraToTarget;
                    tagX = camToTarget.getMeasureX().in(Meter);
                }
            }
         }
    return tagX;
  }  

    public double getTagYDist() {
        double tagY = 0;
        if (camera.isConnected()) {
            List<PhotonPipelineResult> currentUnreadPipeline = camera.getAllUnreadResults();
            if (!currentUnreadPipeline.isEmpty()) {
                PhotonPipelineResult latestTag = currentUnreadPipeline.get(currentUnreadPipeline.size() - 1);
                if (latestTag.hasTargets() || latestTag.getBestTarget() != null) {
                    Transform3d camToTarget = latestTag.getBestTarget().bestCameraToTarget;
                    tagY = camToTarget.getMeasureY().in(Meter);
                }
            }
         }
    return tagY;
  }  
  
  // make state machine that tells this what thte current tag heigh is - when we want to make this work
        // make another state machine that enables flashing led when target is not found 
    //    if (mRobotToCam.getZ() - Constants.VisionConstants.CORAL_APRILTAG_HEIGHT > 0.05) { // tune this value. This could be the ideal value for height
//
   //     }
     //   return PhotonUtils.calculateDistanceToTargetMeters(mRobotToCam.getZ(), Constants.VisionConstants.CORAL_APRILTAG_HEIGHT, 0, tagPitch);
  
    public String getName() {
        if (camera.isConnected()) {
            return camera.getName();
        } else {
            return  "No camera found.";
        }
    }
    public double getTagYaw() {
        double tagYaw = 0;
        double result = 0;
        if (camera.isConnected()) {
            List<PhotonPipelineResult> currentUnreadPipeline = camera.getAllUnreadResults();
            if (!currentUnreadPipeline.isEmpty()) {
                PhotonPipelineResult latestTag = currentUnreadPipeline.get(currentUnreadPipeline.size() - 1);
                if (latestTag.hasTargets() || latestTag.getBestTarget() != null) {
                    tagYaw = latestTag.getBestTarget().bestCameraToTarget.getRotation().getZ();
                    if (tagYaw > 0) {
                        result = tagYaw - Math.PI;
                    } 
                    if (tagYaw < 0) {
                        result = tagYaw + Math.PI;
                    }
                }
            } 
        } 
        return result;
    }    

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        PhotonPipelineResult result = new PhotonPipelineResult();
        if (camera.isConnected()) {
            photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
            var results = camera.getAllUnreadResults();
            if (!results.isEmpty()) {
                return photonPoseEstimator.update(results.get(results.size() - 1));
            } else {
                return photonPoseEstimator.update(result);
            }
        } else {
            return photonPoseEstimator.update(result);
        }
    }
}
