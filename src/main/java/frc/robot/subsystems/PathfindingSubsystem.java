package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PathfindingSubsystem extends SubsystemBase {

// Create the constraints to use while pathfinding
 private final PathConstraints constraint; 
 SwerveRequest.Idle idle;
 
    public PathfindingSubsystem(PathConstraints constraints) {
        constraint = constraints;        
    }

    public Command pathfinder() {
        return runEnd(() -> {
         //   AutoBuilder.pathfindToPose(targetPose, constraint).andThen(AutoBuilder.followPath(PathPlannerPath.fromPathFile("test")));
        }, () -> {
        });
    }

    public PathPlannerPath setPath(String name) {
        PathPlannerPath pathToSet = PathPlannerPath.fromPathFile(name);
        try{
          pathToSet = PathPlannerPath.fromPathFile(name);  
        } catch(Exception e) {
            DriverStation.reportError("Failed to find path!", null);
        }

        return pathToSet;
    }

    public PathConstraints returnConstraints() {
        return constraint;
    }







}
