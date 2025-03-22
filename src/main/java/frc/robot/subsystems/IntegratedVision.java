package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.ConstrainedSolvepnpParams;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.util.Records.TimestampedPose;
import frc.robot.util.Records.TimestampedState;
import frc.robot.util.Records.VisionMeasurement;

public class IntegratedVision extends SubsystemBase {
    private final AprilTagFieldLayout aprilTags = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    private PhotonCamera reef;
    private PhotonCamera feeder;

    private PhotonPoseEstimator reef_global;
    private PhotonPoseEstimator feeder_global;
    private PhotonPoseEstimator reef_local;

    private final Optional<ConstrainedSolvepnpParams> constrainedPnpParams;

    private CommandSwerveDrivetrain dt;

    


    public IntegratedVision(CommandSwerveDrivetrain driver) {
        reef = new PhotonCamera("reef_cam");
        feeder = new PhotonCamera("feeder_cam");

        reef_local = new PhotonPoseEstimator(aprilTags, PoseStrategy.PNP_DISTANCE_TRIG_SOLVE, Constants.Vision.reefRobotToCam);
        reef_global = new PhotonPoseEstimator(aprilTags, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.Vision.reefRobotToCam);
        feeder_global = new PhotonPoseEstimator(aprilTags, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.Vision.feederRobotToCam);
        
        constrainedPnpParams = Optional.of(new ConstrainedSolvepnpParams(true, 0.0));

        dt = driver;
        // photonvision turbo unlocker

    }

   StructPublisher<Pose2d> globalPublisher = NetworkTableInstance.getDefault()
  .getStructTopic("Onboard Pose Estimate", Pose2d.struct).publish();
   StructArrayPublisher<Pose2d> globalArrayPublisher = NetworkTableInstance.getDefault()
  .getStructArrayTopic("Onboard Pose Estimate Array", Pose2d.struct).publish();

  StructPublisher<Pose2d> localPublisher = NetworkTableInstance.getDefault()
  .getStructTopic("Local Pose Estimate", Pose2d.struct).publish();
   StructArrayPublisher<Pose2d> localArrayPublisher = NetworkTableInstance.getDefault()
  .getStructArrayTopic("Local Pose Estimate Array", Pose2d.struct).publish();

    /* 
     *  Add heading data to the reef estimator.
     */

    public void addTrigSolveReference(TimestampedPose tmp) {
        if (reef.isConnected()) {
            reef_local.addHeadingData(tmp.reportedTimestamp(), tmp.reportedPose().getRotation());
        }
    }

    /*
     * Run refresh on the reef Trig Pose estimator, reef Global estimator, and feeder global Estimator.
     * Returns a list where the first VisionMeasurement instance is the local reef estimate, the second instance is
     * the reef global estimate, and the final is the feeder global estimate. 
     * 1st: Local
     * 2nd: Reef-Global
     * 3rd: Feeder-Global
     */

    public List<VisionMeasurement> refresh(TimestampedState state) {

        // Initial initialization of all metrics.
        double reefLocalDistStdDevs = 1000; 
        double reefLocalAngleStdDevs = 1000;
       
        double reefGlobalDistStdDevs = 1000;
        double reefGlobalAngleStdDevs = 1000;

        double feederGlobalDistStdDevs = 1000;
        double feederGlobalAngleStdDevs = 1000;

        Optional <EstimatedRobotPose> estimateInReefGlobalPipeline;
        Optional <EstimatedRobotPose> estimateInReefLocalPipeline;
        Optional <EstimatedRobotPose> estimateInFeederPipeline;

        Pose2d reportedLocalEstimate = new Pose2d();
        double reportedLocalTimestamp = 0; // Ensure no override of existing data. 

        Pose2d reportedReefGlobalEstimate = new Pose2d();
        double reportedReefGlobalTimestamp = 0; // Ensure no override of existing data. 

        Pose2d reportedFeederGlobalEstimate = new Pose2d();
        double reportedFeederGlobalTimestamp = 0; // Ensure no override of existing data. 

        boolean reefReady = reef.isConnected(); // Ensure no robot code failures.
        boolean feederReady = feeder.isConnected(); // Ensure no robot code failures. 

        boolean pushLocal = false;
        boolean pushReefGlobal = false;
        boolean pushFeederGlobal = false;

        // PhotonVision Turbo Button
        NetworkTableInstance.getDefault().getBooleanTopic("/photonvision/use_new_cscore_frametime").publish().set(true);


        if (reefReady) {
            for (PhotonPipelineResult result : reef.getAllUnreadResults()) {
                if (Math.abs(state.reportedState().Speeds.omegaRadiansPerSecond) > Math.PI * 2) continue; // Don't continue loop if the speed of the robot was too great.

                // If disabled, run multitag processing to get an initial pose (UNTESTED)
                reef_local.setPrimaryStrategy(
                    DriverStation.isEnabled() ? PoseStrategy.PNP_DISTANCE_TRIG_SOLVE : PoseStrategy.CONSTRAINED_SOLVEPNP
                ); 

                reef_global.setPrimaryStrategy(
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
                ); 
                
                estimateInReefGlobalPipeline = reef_global.update(result, Optional.empty(), Optional.empty(), constrainedPnpParams);
                estimateInReefLocalPipeline = reef_local.update(result, Optional.empty(), Optional.empty(), constrainedPnpParams);

                if (!(estimateInReefGlobalPipeline.isEmpty() || estimateInReefGlobalPipeline.get().targetsUsed.isEmpty())) {
                    var target = estimateInReefGlobalPipeline.get().targetsUsed.get(0);
                    int id = target.fiducialId;
                    if (!useTag(id)) continue;

                    var tagLocation = aprilTags.getTagPose(id);
                    if (tagLocation.isEmpty()) continue; // Redundant, but every team I saw is using this. 

                    double dist = target.bestCameraToTarget.getTranslation().getNorm();
                    if (dist > 4) continue; // Tune this. Basically if our camera distance is within a set amount, change.
                    // likely larger for a global pose estimate and REALLY tiny for the trigsolve.

                    reefGlobalDistStdDevs = 0.1 * dist * dist;
                    reefGlobalAngleStdDevs = 0.12 * dist * dist;

                    pushReefGlobal = true;

                    if (estimateInReefGlobalPipeline.isPresent() && pushReefGlobal) {
                        reportedReefGlobalEstimate = estimateInReefGlobalPipeline.get().estimatedPose.toPose2d();
                        reportedReefGlobalTimestamp = estimateInReefGlobalPipeline.get().timestampSeconds;
                    }
                }

                if (!(estimateInReefLocalPipeline.isEmpty() || estimateInReefLocalPipeline.get().targetsUsed.isEmpty())) {
                    var target = estimateInReefLocalPipeline.get().targetsUsed.get(0);
                    int id = target.fiducialId;
                    if (!useTag(id)) continue;

                    var tagLocation = aprilTags.getTagPose(id);
                    if (tagLocation.isEmpty()) continue; // Redundant, but every team I saw is using this. 

                    double dist = target.bestCameraToTarget.getTranslation().getNorm();
                    if ((dist > 2)) continue; // Tune this. Basically if our camera distance is within a set amount, change.
                    // likely larger for a global pose estimate and REALLY tiny for the trigsolve.

                    reefLocalDistStdDevs = 0.1 * dist * dist;
                
                    // change angular pose estimation weight because TRIG_SOLVE does NOT use heading data, we don't want to duplicate data.
                    reefLocalAngleStdDevs = !reef_local.getPrimaryStrategy().equals(PoseStrategy.PNP_DISTANCE_TRIG_SOLVE)
                    ? 0.12 * Math.pow(dist, 2)
                    : 1e5;

                    pushLocal = true;

                    if (estimateInReefLocalPipeline.isPresent() && pushLocal) {
                        reportedLocalEstimate = estimateInReefLocalPipeline.get().estimatedPose.toPose2d();
                        reportedLocalTimestamp = estimateInReefLocalPipeline.get().timestampSeconds;
                    }
                }
             }
        }

        if (feederReady) {
            for (PhotonPipelineResult result : feeder.getAllUnreadResults()) {
                if (Math.abs(state.reportedState().Speeds.omegaRadiansPerSecond) > Math.PI * 2) continue; // Don't continue loop if the speed of the robot was too great.

                feeder_global.setPrimaryStrategy(
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
                ); 
                
                estimateInFeederPipeline = feeder_global.update(result, Optional.empty(), Optional.empty(), constrainedPnpParams);

                if (!(estimateInFeederPipeline.isEmpty() || estimateInFeederPipeline.get().targetsUsed.isEmpty())) {
                    var target = estimateInFeederPipeline.get().targetsUsed.get(0);
                    int id = target.fiducialId;
                    if (!useTag(id)) continue;

                    var tagLocation = aprilTags.getTagPose(id);
                    if (tagLocation.isEmpty()) continue; // Redundant, but every team I saw is using this. 

                    double dist = target.bestCameraToTarget.getTranslation().getNorm();
                    if (dist > 4) continue; // Tune this. Basically if our camera distance is within a set amount, change.
                    // likely larger for a global pose estimate and REALLY tiny for the trigsolve.

                    feederGlobalDistStdDevs = 0.1 * dist * dist;
                    feederGlobalAngleStdDevs = 0.12 * dist * dist;

                    pushFeederGlobal = true;

                    if (estimateInFeederPipeline.isPresent() && pushFeederGlobal) {
                        reportedFeederGlobalEstimate = estimateInFeederPipeline.get().estimatedPose.toPose2d();
                        reportedFeederGlobalTimestamp = estimateInFeederPipeline.get().timestampSeconds;
                    }
                }                
            }
        }

        return List.of(
            new VisionMeasurement(new TimestampedPose(reportedLocalEstimate, reportedLocalTimestamp), VecBuilder.fill(reefLocalDistStdDevs, reefLocalDistStdDevs, reefLocalAngleStdDevs)),
            new VisionMeasurement(new TimestampedPose(reportedReefGlobalEstimate, reportedReefGlobalTimestamp), VecBuilder.fill(reefGlobalDistStdDevs, reefGlobalDistStdDevs, reefGlobalAngleStdDevs)),
            new VisionMeasurement(new TimestampedPose(reportedFeederGlobalEstimate, reportedFeederGlobalTimestamp), VecBuilder.fill(feederGlobalDistStdDevs, feederGlobalDistStdDevs, feederGlobalAngleStdDevs))

        );
    }

    // a simple utility function to indicate whether an estimate is to be trusted (based on AprilTag IDs)

    private boolean useTag(int id) {
        return (DriverStation.getAlliance().get() == Alliance.Blue) ? (id >= 17 && id <= 22) : (id >= 6 && id <= 11);
    }

    @Override
    public void periodic() {

        TimestampedState stateAtLoopIteration = new TimestampedState(dt.getState(), Timer.getFPGATimestamp());
        List<VisionMeasurement> currentMeasurements = refresh(stateAtLoopIteration);

        addTrigSolveReference(new TimestampedPose(stateAtLoopIteration.reportedState().Pose, stateAtLoopIteration.reportedTimestamp()));

        dt.updateLocalizedEstimator(currentMeasurements.get(0));
        dt.updateGlobalEstimator(currentMeasurements.get(1));
        dt.updateGlobalEstimator(currentMeasurements.get(2));

        // Logging

        globalPublisher.set(dt.getState().Pose);
        globalArrayPublisher.set(new Pose2d[] {dt.getState().Pose});
    
        localPublisher.set(dt.getLocalizedPose());
        localArrayPublisher.set(new Pose2d[] {dt.getLocalizedPose()});
    }
}
