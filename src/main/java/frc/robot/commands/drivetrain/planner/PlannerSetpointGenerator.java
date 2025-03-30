package frc.robot.commands.drivetrain.planner;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.AutoConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class PlannerSetpointGenerator extends Command {
    
    public CommandSwerveDrivetrain mSwerve;
    public final Pose2d goalPose;

    private PPHolonomicDriveController mDriveController = Constants.AutoConstants.kDriveController;
    private final SwerveRequest.ApplyRobotSpeeds mChassisSpeed;
    private final Timer timer = new Timer();

    private final Debouncer endTriggerDebouncer = new Debouncer(0.04);



    private final BooleanPublisher endTriggerLogger = NetworkTableInstance.getDefault().getTable("logging").getBooleanTopic("PositionPIDEndTrigger").publish();

    public PlannerSetpointGenerator(CommandSwerveDrivetrain mSwerve, Pose2d goalPose) {
        this.mSwerve = mSwerve;
        this.goalPose = goalPose;
        mChassisSpeed = new SwerveRequest.ApplyRobotSpeeds().withDriveRequestType(DriveRequestType.Velocity).withSteerRequestType(SteerRequestType.MotionMagicExpo);
    }

    public static Command generateCommand(CommandSwerveDrivetrain swerve, Pose2d goalPose, Time timeout, boolean flipIt){
        return new PlannerSetpointGenerator(swerve, goalPose).withTimeout(timeout).finallyDo(() -> {
            
            swerve.setControl(
                new SwerveRequest.ApplyRobotSpeeds().withSpeeds(new ChassisSpeeds())
            );


            swerve.setControl(new SwerveRequest.SwerveDriveBrake());
        });
       
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        PathPlannerTrajectoryState goalState = new PathPlannerTrajectoryState();
        goalState.pose = goalPose;


        mSwerve.setControl(
            mChassisSpeed.withSpeeds(
            mDriveController.calculateRobotRelativeSpeeds(
                mSwerve.getState().Pose, goalState
            ))
        );
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();

        System.out.println("Adjustment to align took" + timer.get() + "seconds and interrupted at a certain condition.");
    }

    @Override
    public boolean isFinished() {
        Pose2d diff = mSwerve.getState().Pose.relativeTo(goalPose);

        var rotation = MathUtil.isNear(
            0, 
            diff.getRotation().getRotations(), 
            Constants.AutoConstants.kRotationTolerance.getRotations(),
            0.0,
            1.0);
        
        var position = diff.getTranslation().getNorm() < AutoConstants.kPositionTolerance.in(Units.Meters);

        var speed = mSwerve.getSpeedAsDouble() < Constants.AutoConstants.kSpeedTolerance.in(Units.MetersPerSecond);

        return endTriggerDebouncer.calculate(
            rotation && position && speed
        );
    }
}