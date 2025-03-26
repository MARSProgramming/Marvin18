package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Dynamic;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.constants.Constants;
import frc.robot.constants.DynamicConstants;
import frc.robot.constants.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.util.MoreMath;
import frc.robot.util.Records.TimestampedState;
import frc.robot.util.Records.VisionMeasurement;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    public int selectedSide = 1;
    Pose2d desiredPose2d = Pose2d.kZero;

    private SwerveDrivePoseEstimator localizedPoseEstimator;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    /* Swerve requests for robot-centric pathfollowing */
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    /* Swerve requests for robot-centric alignment */
    private final SwerveRequest.ApplyRobotSpeeds m_alignApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();
    private final SwerveRequest.ApplyRobotSpeeds mChassiSpeeds = new SwerveRequest.ApplyRobotSpeeds();


    /*
     * SysId routine for characterizing translation. This is used to find PID gains
     * for the drive motors.
     */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> setControl(m_translationCharacterization.withVolts(output)),
                    null,
                    this));
    /*
     * SysId routine for characterizing steer. This is used to find PID gains for
     * the steer motors.
     */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(7), // Use dynamic voltage of 7 V
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    volts -> setControl(m_steerCharacterization.withVolts(volts)),
                    null,
                    this));
    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle
     * HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on
     * importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    /* This is in radians per second², but SysId only supports "volts per second" */
                    Volts.of(Math.PI / 6).per(Second),
                    /* This is in radians per second, but SysId only supports "volts" */
                    Volts.of(Math.PI),
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> {
                        /* output is actually radians per second, but SysId only supports "volts" */
                        setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                        /* also log the requested output for SysId */
                        SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
                    },
                    null,
                    this));

    public Command setSide(int side) {
        return runOnce(() -> {
            selectedSide = side;
        });
    }

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineSteer;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules             Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();


        localizedPoseEstimator = new SwerveDrivePoseEstimator(getKinematics(), getState().Pose.getRotation(),
                getState().ModulePositions, new Pose2d(),
                VecBuilder.fill(1, 1, 3),
                VecBuilder.fill(1, 1, 3));

    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve
     *                                  drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz
     *                                  on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry
     *                                  calculation
     *                                  in the form [x, y, theta]ᵀ, with units in
     *                                  meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision
     *                                  calculation
     *                                  in the form [x, y, theta]ᵀ, with units in
     *                                  meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation,
                modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        configureAutoBuilder();
    }

    /**
     * Returns a command that applies the specified control request to this swerve
     * drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command seedCentric() {
        return runOnce(() -> this.seedFieldCentric());
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public void drive(ChassisSpeeds speeds) {
        setControl(mChassiSpeeds.withSpeeds(speeds));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply
         * it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts
         * mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is
         * disabled.
         * This ensures driving behavior doesn't change until an explicit disable event
         * occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                                ? kRedAlliancePerspectiveRotation
                                : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
            });
        }

        localizedPoseEstimator.updateWithTime(Timer.getFPGATimestamp(), getState().Pose.getRotation(),
                getState().ModulePositions);

    }



    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the
     * odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision
     *                              camera.
     * @param timestampSeconds      The timestamp of the vision measurement in
     *                              seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the
     * odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters    The pose of the robot as measured by the
     *                                 vision camera.
     * @param timestampSeconds         The timestamp of the vision measurement in
     *                                 seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose
     *                                 measurement
     *                                 in the form [x, y, theta]ᵀ, with units in
     *                                 meters and radians.
     */
    @Override
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds),
                visionMeasurementStdDevs);
    }

    private void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                    () -> getState().Pose, // Supplier of current robot pose
                    this::resetAllEstimatorPoses, // Consumer for seeding pose against auto
                    () -> getState().Speeds, // Supplier of current robot speeds
                    // Consumer of ChassisSpeeds and feedforwards to drive the robot
                    (speeds, feedforwards) -> setControl(
                            m_pathApplyRobotSpeeds.withSpeeds(speeds)
                                    .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                                    .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                                    .withDriveRequestType(
                                            com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.Velocity)),
                    new PPHolonomicDriveController(
                            // PID constants for translation
                            new PIDConstants(10, 0, 0),
                            // PID constants for rotation
                            new PIDConstants(11, 0, 0)),
                    config,
                    // Assume the path needs to be flipped for Red vs Blue, this is normally the
                    // case
                    () -> {
                        var alliance = DriverStation.getAlliance();
                        if (alliance.isPresent()) {
                            return alliance.get() == DriverStation.Alliance.Red;
                        }
                        return false;
                    },
                    this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder",
                    ex.getStackTrace());
        }
    }

    private HolonomicDriveController mHolonomicDriveController = new HolonomicDriveController(
            new PIDController(Constants.PathPlannerAuto.holonomicXkP, Constants.PathPlannerAuto.holonomicXkI,
                    Constants.PathPlannerAuto.holonomicXkD),
            new PIDController(Constants.PathPlannerAuto.holonomicYkP, Constants.PathPlannerAuto.holonomicYkI,
                    Constants.PathPlannerAuto.holonomicYkD),
            new ProfiledPIDController(Constants.PathPlannerAuto.holonomicOkP, Constants.PathPlannerAuto.holonomicOkI,
                    Constants.PathPlannerAuto.holonomicOkD,
                    new TrapezoidProfile.Constraints(Constants.PathPlannerAuto.holonomicOMaxVelocity,
                            Constants.PathPlannerAuto.holonomicOMaxAcceleration)));

    public HolonomicDriveController getDrivePathController() {
        return mHolonomicDriveController;
    }

    public ChassisSpeeds getFieldRelativeSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(getState().Speeds, getState().Pose.getRotation());
    }

    public Command driveToPose() {
        return AutoBuilder.pathfindToPose(new Pose2d(15, 2, Rotation2d.fromDegrees(0)),
                new PathConstraints(LinearVelocity.ofBaseUnits(1.0, MetersPerSecond),
                        LinearAcceleration.ofBaseUnits(0.5, MetersPerSecondPerSecond),
                        AngularVelocity.ofBaseUnits(360, DegreesPerSecond),
                        AngularAcceleration.ofBaseUnits(540, DegreesPerSecondPerSecond)));
    }

    public double getSpeedAsDouble() {
        ChassisSpeeds fieldVelocity = getFieldRelativeSpeeds();
        return Math.sqrt(fieldVelocity.vxMetersPerSecond * fieldVelocity.vxMetersPerSecond
                + fieldVelocity.vyMetersPerSecond * fieldVelocity.vyMetersPerSecond);
    }

    public void updateLocalizedEstimator(VisionMeasurement measure) {
        Matrix<N3, N1> stdevsToUpdate = measure.stDevs();
        localizedPoseEstimator.setVisionMeasurementStdDevs(stdevsToUpdate);
        localizedPoseEstimator.addVisionMeasurement(measure.pose().reportedPose(), measure.pose().reportedTimestamp());
    }

    public Pose2d getLocalizedPose() {
        return localizedPoseEstimator.getEstimatedPosition();
    }

    public void resetLocalizedPose(VisionMeasurement measure) {
        localizedPoseEstimator.resetPose(measure.pose().reportedPose());
    }

    public void resetAllEstimatorPoses(Pose2d pose) {
        localizedPoseEstimator.resetPose(pose);
        resetPose(pose);
    }

    public void updateGlobalEstimator(VisionMeasurement measure) {
        Matrix<N3, N1> stdevsToUpdate = measure.stDevs();
        setVisionMeasurementStdDevs(stdevsToUpdate);
        addVisionMeasurement(measure.pose().reportedPose(), measure.pose().reportedTimestamp());
    }

    public ChassisSpeeds calculateRobotRelative(Pose2d pose) {
        desiredPose2d = pose;
        return Constants.AlignmentConstants.kTeleoperatedAlignmentController.calculate(getState().Pose, pose, 0, pose.getRotation());
    }

    public AngularVelocity calculateRobotRelativeRot(Rotation2d desiredRot) {
        double calculatedYaw = Constants.AlignmentConstants.
        kTeleoperatedAlignmentController.
        getThetaController().
        calculate(getState().Pose.getRotation().getRadians(), desiredRot.getRadians());

        // clamp
        calculatedYaw = MathUtil.clamp(calculatedYaw, Constants.AlignmentConstants.kMaximumRotSpeed.in(Units.RadiansPerSecond), Constants.AlignmentConstants.kMaximumRotSpeed.in(Units.RadiansPerSecond));

        return Units.RadiansPerSecond.of(calculatedYaw);
    }

    public void rotationalAlign(Pose2d desiredTarget, LinearVelocity xVelocity, LinearVelocity yVelocity) {
        // Rotational-only auto-align
        drive(
            new ChassisSpeeds(
                xVelocity.in(Units.MetersPerSecond),
                yVelocity.in(Units.MetersPerSecond),
                calculateRobotRelativeRot(desiredTarget.getRotation()).in(Units.RadiansPerSecond)
            )       
        );
        
        
    }

    
    public void integratedAutoAlignment(Distance distFromTarg, Pose2d desiredTarg, LinearVelocity xVelocity, LinearVelocity yVelocity, AngularVelocity rVelocity, double ElevatorMulti) {
        if (distFromTarg.gte(Constants.AlignmentConstants.kMinimumXYAlignDistance)) {
            rotationalAlign(desiredTarg, xVelocity, yVelocity);
        } else {
            // limit speed based on elevator height
            ChassisSpeeds desiredSpeeds = calculateRobotRelative(desiredTarg);
            LinearVelocity linearSpeedLimit = Constants.AlignmentConstants.TeleoperatedMaximumVelocity.times(ElevatorMulti);
            AngularVelocity angularVelocityLimit = Constants.AlignmentConstants.kMaximumRotSpeed.times(ElevatorMulti);

            // when not in autonomous, clamp speed for better driver control and unanimity with elevator height
            if (!RobotState.isAutonomous()) {
                // Check to see if the desired speeds conflict with teleoperated "flow"
                if (desiredSpeeds.vxMetersPerSecond > linearSpeedLimit.in(Units.MetersPerSecond) ||
                desiredSpeeds.vyMetersPerSecond > linearSpeedLimit.in(Units.MetersPerSecond) ||
                desiredSpeeds.omegaRadiansPerSecond > angularVelocityLimit.in(Units.RadiansPerSecond)) {

                    // clamp speeds
                    desiredSpeeds.vxMetersPerSecond = MathUtil.clamp(desiredSpeeds.vxMetersPerSecond, 0,
                    linearSpeedLimit.in(MetersPerSecond));
                    desiredSpeeds.vyMetersPerSecond = MathUtil.clamp(desiredSpeeds.vyMetersPerSecond, 0,
                    linearSpeedLimit.in(MetersPerSecond));
                    desiredSpeeds.omegaRadiansPerSecond = MathUtil.clamp(desiredSpeeds.omegaRadiansPerSecond, 0,
                    angularVelocityLimit.in(RadiansPerSecond));

                }
            }

            drive(desiredSpeeds);
        }
    }


    public Pose2d getReefRequest(boolean leftSideRequested, int level) {
        // return (DriverStation.getAlliance().get() == Alliance.Blue) ? (id >= 17 && id <= 22) : (id >= 6 && id <= 11);
         Pose2d poseToGet = MoreMath.getNearest(getState().Pose, (DriverStation.getAlliance().get() == Alliance.Blue) ? (Constants.VisionFiducials.BLUE_CORAL_TAGS) : (Constants.VisionFiducials.RED_CORAL_TAGS));
        // new Transform2d(x, y, rot);
         if (level == 4) {
             return poseToGet.transformBy(
                 new Transform2d(
                     (leftSideRequested) ? (DynamicConstants.AlignTransforms.LeftXL4) : (DynamicConstants.AlignTransforms.RightXL4),
                     (leftSideRequested) ? (DynamicConstants.AlignTransforms.LeftYL4) : (DynamicConstants.AlignTransforms.LeftYL4),
                     new Rotation2d((leftSideRequested) ? (DynamicConstants.AlignTransforms.LeftRot) : (DynamicConstants.AlignTransforms.RightRot))));
         }
 
         if (level == 3) {
             return poseToGet.transformBy(
                 new Transform2d(
                     (leftSideRequested) ? (DynamicConstants.AlignTransforms.LeftXL3) : (DynamicConstants.AlignTransforms.RightXL3),
                     (leftSideRequested) ? (DynamicConstants.AlignTransforms.LeftYL3) : (DynamicConstants.AlignTransforms.RightYL3),
                     new Rotation2d((leftSideRequested) ? (DynamicConstants.AlignTransforms.LeftRot) : (DynamicConstants.AlignTransforms.RightRot))));
         }
 
         if (level == 2) {
             return poseToGet.transformBy(
                 new Transform2d(
                     (leftSideRequested) ? (DynamicConstants.AlignTransforms.LeftXL2) : (DynamicConstants.AlignTransforms.RightXL2),
                     (leftSideRequested) ? (DynamicConstants.AlignTransforms.LeftYL2) : (DynamicConstants.AlignTransforms.RightYL2),
                     new Rotation2d((leftSideRequested) ? (DynamicConstants.AlignTransforms.LeftRot) : (DynamicConstants.AlignTransforms.RightRot))));
         }
 
         if (level == 1) {
             return poseToGet.transformBy(
                 new Transform2d(
                     (leftSideRequested) ? (DynamicConstants.AlignTransforms.LeftXL1) : (DynamicConstants.AlignTransforms.RightXL1),
                     (leftSideRequested) ? (DynamicConstants.AlignTransforms.LeftYL1) : (DynamicConstants.AlignTransforms.RightYL1),
                     new Rotation2d((leftSideRequested) ? (DynamicConstants.AlignTransforms.LeftRot) : (DynamicConstants.AlignTransforms.RightRot))));
         }
 
         // if no conditions are met perform center alignment (you can just feed level = -1 lol)
         else {
             return poseToGet.transformBy(
                 new Transform2d(
                     DynamicConstants.AlignTransforms.CentX, DynamicConstants.AlignTransforms.CentY, new Rotation2d(DynamicConstants.AlignTransforms.CentRot))); 
         }
     }


     public void integratedReefAlignment(boolean leftBranch, int level, LinearVelocity xVelocity, LinearVelocity yVelocity, AngularVelocity rVelocity, double ElevatorMulti, Distance maxDistance) {
        Pose2d desiredReef = getReefRequest(leftBranch, level);
        Distance distFromReef = Units.Meters.of(getState().Pose.getTranslation().getDistance(desiredReef.getTranslation()));

        integratedAutoAlignment(distFromReef, desiredReef, xVelocity, yVelocity, rVelocity, ElevatorMulti);
     }

     public boolean isAtRotation(Rotation2d desiredRotation) {
        return (getState().Pose.getRotation().getMeasure()
            .compareTo(desiredRotation.getMeasure().minus(Constants.AlignmentConstants.kRotTolerance)) > 0) &&
            getState().Pose.getRotation().getMeasure()
                .compareTo(desiredRotation.getMeasure().plus(Constants.AlignmentConstants.kRotTolerance)) < 0;
      }
    
      public boolean isAtPosition(Pose2d desiredPose2d) {
        return Units.Meters
            .of(getState().Pose.getTranslation().getDistance(desiredPose2d.getTranslation()))
            .lte(Constants.AlignmentConstants.kPointTolerance);
      }
    
      public boolean isAligned() {
        return (desiredPose2d.getTranslation().getDistance(
            getState().Pose.getTranslation()) <= Constants.AlignmentConstants.kAlignmentTolerance.in(Units.Meters))
            && isAtRotation(desiredPose2d.getRotation());
      }
    
      public boolean atPose(Pose2d desiredPose) {
        return isAtRotation(desiredPose.getRotation()) && isAtPosition(desiredPose);
      }

      // for logging / dashboard odom

      public Pose2d getCurrentPoseToAlign() {
        return desiredPose2d;
      }

}