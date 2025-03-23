package frc.robot.constants;

import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Seconds;


import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;

public final class Constants {
    public static class AltSwerveConstants {
        public static final Distance TRACK_WIDTH = Distance.ofBaseUnits(22.75, Inches); // Distance between Left & Right
                                                                                        // Wheels
        public static final Distance WHEELBASE = Distance.ofBaseUnits(22.75, Inches); // Distance between Front & Back
                                                                                      // Wheels
        public static final double INPUT_MULT = 0.8; // Reduce maximum applied speed for better driveability.
    }

    public static class ElevatorSetpointConfigs {
        public static final double ELEVATOR_DEADZONE_DIST = 0.02;
        public static final double INCHES_PER_ROTATION = 3; // in testing the chassis moves 3 inches per full rotation.
        public static final double ELEVATOR_FORWARD_LIMIT = 97; // for master.
        public static final double ELEVATOR_REVERSE_LIMIT = 0; // for master.
        public static final double ELEVATOR_ROTATIONS_DEADZONE = 0.1;
        public static final double ELEVATOR_SAFE_POSITION = 5.5;
        public static final double ELEVATOR_SCORE_POSITION = 6;
    }

    public static class PathPlannerAuto {
        public static final double holonomicXkP = 5;
        public static final double holonomicXkI = 0;
        public static final double holonomicXkD = 0;
        public static final double holonomicYkP = 5;
        public static final double holonomicYkI = 0;
        public static final double holonomicYkD = 0;
        public static final double holonomicOkP = 3.0;
        public static final double holonomicOkI = 0.0;
        public static final double holonomicOkD = 0.0;
        public static final double holonomicOMaxVelocity = 5;
        public static final double holonomicOMaxAcceleration = 5;
    }

    public static class DIO_IDS {
        public static final int CLIMB_LIMIT = 0;
        public static final int CORAL_ARM_LIMIT = 4;

        public static final int BUCKET_BEAMBREAK = 1;
        public static final int INTAKE_BEAMBREAK = 2;
    }

    public static class PWM_IDS {
        public static final int SERVO = 4;
        public static final int ELEC_BAY_LED = 0;
        public static final int CORAL_INTAKE_LED = 1;
        public static final int RIGHT_CLIMB_LED = 2;
        public static final int LEFT_CLIMB_LED = 3;
        public static final int TOP_CLIMB_LED = 4;
    }

    public static class CAN_IDS {
        public static final String SYSTEM_CAN_NAME = "system"; // CANBUS for system.
        public static final String DRIVETRAIN_CAN_NAME = "drivetrain"; // CANBUS for drivetrain.
        public static final CANBus DrivetrainAndClimbBus = new CANBus("CAN-2", "./logs/example.hoot");

        public static final int SYSTEM_CAN_ID = 21;

        public static class CORAL_MECHANISM {
            public static final int CORAL_BUCKET_ROTATE = 1;
            public static final int AGITATOR = 13;
            public static final int CORAL_INTAKE = 12;
        }

        public static class ALGAE_MECHANISM {
            public static final int ALGAE_MECH_MC = 11;
        }

        public static final class DRIVETRAIN {
            public static final int FRONT_LEFT_DRIVE = 2;
            public static final int FRONT_LEFT_STEER = 3;
            public static final int BACK_LEFT_DRIVE = 4;
            public static final int BACK_LEFT_STEER = 5;
            public static final int FRONT_RIGHT_DRIVE = 8;
            public static final int FRONT_RIGHT_STEER = 9;
            public static final int BACK_RIGHT_DRIVE = 6;
            public static final int BACK_RIGHT_STEER = 7;

            public static final int PIGEON_LEFT = 20;
            public static final int PIGEON_RIGHT = 40; // This is incorrect.
        }

        public static final class ELEVATOR {
            public static final int ELEVATOR_MASTER = 18; // right
            public static final int ELEVATOR_FOLLOWER = 19; // left
        }

    }

    public static final class AutoConstants {
        public static final PIDConstants AUTO_DRIVE_PID = new PIDConstants(
            2.5,
                0,
                0);
        public static final PIDConstants AUTO_STEER_PID = new PIDConstants(
                2.5,
                0,
                0);

        public static final PPHolonomicDriveController kDriveController = new PPHolonomicDriveController(
                AUTO_DRIVE_PID,
                AUTO_STEER_PID);

        public static final Rotation2d kRotationTolerance = Rotation2d.fromDegrees(3);
        public static final Distance kPositionTolerance = Centimeter.of(1.5);
        public static final LinearVelocity kSpeedTolerance = InchesPerSecond.of(2);

        public static final Time kEndTriggerDebounce = Seconds.of(0.04);
        public static final PathConstraints kPathConstraints = new PathConstraints(5.50, 5, 1/2 * Math.PI, 1 * Math.PI); // The constraints for this path.
        public static final PathConstraints kSlowPathConstraints = new PathConstraints(0.3, 0.1, 1/5 * Math.PI, 0.2 * Math.PI); // The constraints for this path.
        public static final Time kAlignmentAdjustmentTimeout = Seconds.of(0.075);

    }


    public static class FieldConstants {
        // tune this
        public static final double FIELD_WIDTH = 16.541;
        public static final double FIELD_LENGTH = 8.211;
        public static final double REEF_APRILTAG_HEIGHT = 6.875; // feet? figure out units
        public static final double PROCCESSOR_APRILTAG_HEIGHT = 45.875;
        public static final double CORAL_APRILTAG_HEIGHT = 53.25;

        public static final Pose2d REEF_A = new Pose2d(2.860, 4.187, Rotation2d.fromDegrees(0));
        public static final Pose2d REEF_B = new Pose2d(2.860, 3.857, Rotation2d.fromDegrees(0));
        public static final Pose2d REEF_C = new Pose2d(3.527, 2.694, Rotation2d.fromDegrees(60));
        public static final Pose2d REEF_D = new Pose2d(3.813, 2.535, Rotation2d.fromDegrees(60));
        public static final Pose2d REEF_E = new Pose2d(5.160, 2.529, Rotation2d.fromDegrees(120));
        public static final Pose2d REEF_F = new Pose2d(5.445, 2.694, Rotation2d.fromDegrees(120));
        public static final Pose2d REEF_G = new Pose2d(6.119, 3.857, Rotation2d.fromDegrees(180));
        public static final Pose2d REEF_H = new Pose2d(6.119, 4.187, Rotation2d.fromDegrees(180));
        public static final Pose2d REEF_I = new Pose2d(5.452, 5.343, Rotation2d.fromDegrees(-120));
        public static final Pose2d REEF_J = new Pose2d(5.166, 5.527, Rotation2d.fromDegrees(-120));
        public static final Pose2d REEF_K = new Pose2d(3.826, 5.508, Rotation2d.fromDegrees(-60));
        public static final Pose2d REEF_L = new Pose2d(3.534, 5.368, Rotation2d.fromDegrees(-60));
    }

    public static class VisionFiducials {
        // F,B,R,L with respect to their driver stations.
        public static final int RED_RIGHT_FEEDER_TAG = 1;
        public static final int RED_LEFT_FEEDER_TAG = 2;
        public static final int BLUE_RIGHT_FEEDER_TAG = 13;
        public static final int BLUE_LEFT_FEEDER_TAG = 12;

        public static final int RED_PROCESSOR_TAG = 3;
        public static final int BLUE_PROCESSOR_TAG = 16;

        public static final int BLUE_BACK_CLIMBING_TAG = 4;
        public static final int RED_FRONT_CLIMBING_TAG = 5;
        public static final int BLUE_FRONT_CLIMBING_TAG = 14;
        public static final int RED_BACK_CLIMBING_TAG = 15;

        public static final int RED_FL_CORAL_TAG = 6;
        public static final int RED_F_CORAL_TAG = 7;
        public static final int RED_FR_CORAL_TAG = 8;
        public static final int RED_BL_CORAL_TAG = 9;
        public static final int RED_B_CORAL_TAG = 10;
        public static final int RED_BR_CORAL_TAG = 11;

        public static final int BLUE_FR_CORAL_TAG = 17;
        public static final int BLUE_F_CORAL_TAG = 18;
        public static final int BLUE_FL_CORAL_TAG = 19;
        public static final int BLUE_BL_CORAL_TAG = 20;
        public static final int BLUE_B_CORAL_TAG = 21;
        public static final int BLUE_BR_CORAL_TAG = 22;

        public static final int[] RED_CORAL_TAGS = { 6, 7, 8, 9, 10, 11 };
        public static final int[] BLUE_CORAL_TAGS = { 17, 18, 19, 20, 21, 22 };
        public static final int[] RED_FEEDER_TAGS = { 1, 2 };
        public static final int[] BLUE_FEEDER_TAGS = { 12, 13 };
        public static final int[] RED_SIDE_CLIMB_TAGS = { 4, 5 };
        public static final int[] BLUE_SIDE_CLIMB_TAGS = { 14, 15 };
        public static final int[] PROCESSOR_TAGS = { 1, 15 };
    }

    public static class Vision {
        public static final String reefCameraName = "reef_cam";
        public static final String feederCameraName = "feeder_cam";

        public static final Transform3d feederRobotToCam = new Transform3d(Inches.of(1.48),
        Inches.of(-10.31), Inches.of(17.54), new Rotation3d(Degrees.of(0),
        Degrees.of(-30), Degrees.of(-90)));

        public static final Transform3d reefRobotToCam = new Transform3d(Inches.of(.35), Inches.of(9.71), Inches.of(20.48),
        new Rotation3d(Degrees.of(0), Degrees.of(10), Degrees.of(90)));



        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout
                .loadField(AprilTagFields.kDefaultField);

        // The standard deviations of our vision estimated poses, which affect
        // correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(1, 1, 2);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    }
}