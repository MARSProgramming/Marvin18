package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;


public class IntegratedVision extends SubsystemBase {
    private final AprilTagFieldLayout aprilTags = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    private PhotonCamera reef;
    private PhotonCamera feeder;
    private CommandSwerveDrivetrain dt;

    private PhotonPoseEstimator reef_global;
    private PhotonPoseEstimator feeder_global;

    private double reefGlobalDistStdDevs = 1000;
    private double reefGlobalAngleStdDevs = 1000;

    private double feederGlobalDistStdDevs = 1000;
    private double feederGlobalAngleStdDevs = 1000;


    private Optional <EstimatedRobotPose> estimateInReefGlobalPipeline;
    private Optional <EstimatedRobotPose> estimateInFeederPipeline;


    private Pose2d reportedReefGlobalEstimate;
    private double reportedReefGlobalTimestamp = 0; // Ensure no override of existing data. 

    private Pose2d reportedFeederGlobalEstimate;
    private double reportedFeederGlobalTimestamp = 0; // Ensure no override of existing data. 

    private boolean reefReady = false; // Ensure no robot code failures.
    private boolean feederReady = false; // Ensure no robot code failures. 

    private boolean pushReefGlobal = false;
    private boolean pushFeederGlobal = false;
    


    public IntegratedVision(CommandSwerveDrivetrain driver) {
        reef = new PhotonCamera("reef_cam");
        feeder = new PhotonCamera("feeder_cam");

        reef_global = new PhotonPoseEstimator(aprilTags, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.Vision.reefRobotToCam);
        feeder_global = new PhotonPoseEstimator(aprilTags, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.Vision.feederRobotToCam);
        
        feeder_global.setPrimaryStrategy(
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
        ); 

        reef_global.setPrimaryStrategy(
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
        ); 

        reef_global.setMultiTagFallbackStrategy(
            PoseStrategy.LOWEST_AMBIGUITY
        ); 

        feeder_global.setMultiTagFallbackStrategy(
            PoseStrategy.LOWEST_AMBIGUITY
        ); 

        reportedReefGlobalEstimate = Pose2d.kZero;
        reportedFeederGlobalEstimate = Pose2d.kZero;

        dt = driver;
    }

   StructPublisher<Pose2d> globalPublisher = NetworkTableInstance.getDefault()
  .getStructTopic("Onboard Pose Estimate", Pose2d.struct).publish();
   StructArrayPublisher<Pose2d> globalArrayPublisher = NetworkTableInstance.getDefault()
  .getStructArrayTopic("Onboard Pose Estimate Array", Pose2d.struct).publish();

  StructPublisher<Pose2d> localPublisher = NetworkTableInstance.getDefault()
  .getStructTopic("Alternative Pose Estimate", Pose2d.struct).publish();
   StructArrayPublisher<Pose2d> localArrayPublisher = NetworkTableInstance.getDefault()
  .getStructArrayTopic("Alternative Pose Estimate Array", Pose2d.struct).publish();


  /*
   * Reset all metrics
   */

    public void resetMetrics() {
               
        reefGlobalDistStdDevs = 1000;
        reefGlobalAngleStdDevs = 1000;

        feederGlobalDistStdDevs = 1000;
        feederGlobalAngleStdDevs = 1000;

        reportedReefGlobalEstimate = Pose2d.kZero;
        reportedReefGlobalTimestamp = 0; // Ensure no override of existing data. 

        reportedFeederGlobalEstimate =  Pose2d.kZero;
        reportedFeederGlobalTimestamp = 0; // Ensure no override of existing data. 

        reefReady = false; // Ensure no robot code failures.
        feederReady = false; // Ensure no robot code failures. 

        pushReefGlobal = false;
        pushFeederGlobal = false;
    }
    /*
     * Run refresh on the reef Trig Pose estimator, reef Global estimator, and feeder global Estimator.
     * Returns a list where the first VisionMeasurement instance is the local reef estimate, the second instance is
     * the reef global estimate, and the final is the feeder global estimate. 
     * 1st: Local
     * 2nd: Reef-Global
     * 3rd: Feeder-Global
     */

    public void refresh() {

        // Initial initialization of all metrics.
        reefReady = reef.isConnected();
        feederReady = feeder.isConnected();


        // PhotonVision Turbo Button


        if (reefReady) {
            for (PhotonPipelineResult result : reef.getAllUnreadResults()) {
                if (Math.abs(dt.getState().Speeds.omegaRadiansPerSecond) > Math.PI * 2) continue; // Don't continue loop if the speed of the robot was too great.

                // If disabled, run multitag processing to get an initial pose (UNTESTED)
                
                estimateInReefGlobalPipeline = reef_global.update(result);
                boolean ignore = false;

                if (!(estimateInReefGlobalPipeline.isEmpty())) {
                    for (PhotonTrackedTarget targ : estimateInReefGlobalPipeline.get().targetsUsed) {
                        if (targ.getPoseAmbiguity() > .2) {
                            ignore = true;
                        }
                    }
                }

                if (ignore) continue;

                // new, experimental logic: Don't allow us to process poses where there are more than 2 tags fed into the estimate. We can change this number but it may be ideal.
                if (!(estimateInReefGlobalPipeline.isEmpty() || estimateInReefGlobalPipeline.get().targetsUsed.isEmpty())) {
                    var target = estimateInReefGlobalPipeline.get().targetsUsed.get(0);
                    if (target.getPoseAmbiguity() > 0.2) continue;
                    int id = target.fiducialId;
                    if (!useTag(id)) continue;

                    // experimental - removed unecessary "area check" logic 
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
             }
        }

        if (feederReady) {
            for (PhotonPipelineResult result : feeder.getAllUnreadResults()) {
                if (Math.abs(dt.getState().Speeds.omegaRadiansPerSecond) > Math.PI * 2) continue; // Don't continue loop if the speed of the robot was too great.
                estimateInFeederPipeline = feeder_global.update(result);

                boolean ignore = false;
                if (!(estimateInFeederPipeline.isEmpty())) {
                    for (PhotonTrackedTarget targ : estimateInFeederPipeline.get().targetsUsed) {
                        if (targ.getPoseAmbiguity() > .2) {
                            ignore = true;
                        }
                    }
                }

                if (ignore) continue;

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

        if (pushReefGlobal == true) {
            dt.setVisionMeasurementStdDevs(VecBuilder.fill(reefGlobalDistStdDevs, reefGlobalDistStdDevs, reefGlobalAngleStdDevs));
            dt.addVisionMeasurement(reportedReefGlobalEstimate, reportedReefGlobalTimestamp);    
        }

        if (pushFeederGlobal == true) {
           dt.setVisionMeasurementStdDevs(VecBuilder.fill(feederGlobalDistStdDevs, feederGlobalDistStdDevs, feederGlobalAngleStdDevs));
           dt.addVisionMeasurement(reportedFeederGlobalEstimate, reportedFeederGlobalTimestamp);    
        }

        // reset variables each loop
        resetMetrics();

    }

    // a simple utility function to indicate whether an estimate is to be trusted (based on AprilTag IDs)

    private boolean useTag(int id) {
        return (DriverStation.getAlliance().get() == Alliance.Blue) ? (id >= 17 && id <= 22) : (id >= 6 && id <= 11);
    }

    @Override
    public void periodic() {

        refresh();
 

        // Logging
        globalPublisher.set(dt.getState().Pose);
        globalArrayPublisher.set(new Pose2d[] {dt.getState().Pose});
       localPublisher.set(dt.getLocalizedPose());
       localArrayPublisher.set(new Pose2d[] {dt.getLocalizedPose()});

    }
}
