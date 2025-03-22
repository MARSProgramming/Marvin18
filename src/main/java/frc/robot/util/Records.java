package frc.robot.util;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class Records {
    public static final record VisionMeasurement(TimestampedPose pose, Matrix<N3, N1> stDevs) {

        /*
         * Represents a measurement from vision to apply to a pose estimator.
         */
    }

    public static final record TimestampedPose(Pose2d reportedPose, double reportedTimestamp) {
        /*
         * Represents a timestamped pose.
         */
    }
}
