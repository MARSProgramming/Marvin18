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

    public void refresh() {

        // Initial initialization of all metrics.
       
        double reefGlobalDistStdDevs = 1000;
        double reefGlobalAngleStdDevs = 1000;

        double feederGlobalDistStdDevs = 1000;
        double feederGlobalAngleStdDevs = 1000;

        Optional <EstimatedRobotPose> estimateInReefGlobalPipeline;
        Optional <EstimatedRobotPose> estimateInFeederPipeline;


        Pose2d reportedReefGlobalEstimate = Pose2d.kZero;
        double reportedReefGlobalTimestamp = 0; // Ensure no override of existing data. 

        Pose2d reportedFeederGlobalEstimate =  Pose2d.kZero;
        double reportedFeederGlobalTimestamp = 0; // Ensure no override of existing data. 

        boolean reefReady = reef.isConnected(); // Ensure no robot code failures.
        boolean feederReady = feeder.isConnected(); // Ensure no robot code failures. 

        boolean pushReefGlobal = false;
        boolean pushFeederGlobal = false;

        feeder_global.setPrimaryStrategy(
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
        ); 

        reef_global.setPrimaryStrategy(
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
        ); 
        // PhotonVision Turbo Button


        if (reefReady) {
            for (PhotonPipelineResult result : reef.getAllUnreadResults()) {
                if (Math.abs(dt.getState().Speeds.omegaRadiansPerSecond) > Math.PI * 2) continue; // Don't continue loop if the speed of the robot was too great.

                // If disabled, run multitag processing to get an initial pose (UNTESTED)
                
                estimateInReefGlobalPipeline = reef_global.update(result, Optional.empty(), Optional.empty(), constrainedPnpParams);

                if (!(estimateInReefGlobalPipeline.isEmpty() || estimateInReefGlobalPipeline.get().targetsUsed.isEmpty())) {
                    var target = estimateInReefGlobalPipeline.get().targetsUsed.get(0);
                    int id = target.fiducialId;
                    if (!useTag(id)) continue;
                    // experimental - reject vision updates if the target is smaller than 20% of the cameraspace
                    // this may help us a lot. no vision updates when we are too far away
                    if (target.getArea() < 0.2) continue; 

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

        if (pushReefGlobal == true) {
            dt.setVisionMeasurementStdDevs(VecBuilder.fill(reefGlobalDistStdDevs, reefGlobalDistStdDevs, reefGlobalAngleStdDevs));
            dt.addVisionMeasurement(reportedReefGlobalEstimate, reportedReefGlobalTimestamp);    
        }

        if (pushFeederGlobal == true) {
            dt.updateLocalizedEstimator(reportedFeederGlobalEstimate, reportedFeederGlobalTimestamp, VecBuilder.fill(feederGlobalDistStdDevs, feederGlobalDistStdDevs, feederGlobalAngleStdDevs));
        }

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
