package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Distance;

public final class Constants {
    public static class AltSwerveConstants {
        public static final Distance TRACK_WIDTH = Distance.ofBaseUnits(22.75, Inches); // Distance between Left & Right Wheels
        public static final Distance WHEELBASE = Distance.ofBaseUnits(22.75, Inches);   // Distance between Front & Back Wheels
        public static final double INPUT_MULT = 0.8; // Reduce maximum applied speed for better driveability.
    }

    public static class DIO_IDS {
        public static final int CLIMB_RIGHT_LIMIT = 0;
        public static final int LEFT_RIGHT_LIMIT = 4;
        public static final int CORAL_ARM_LIMIT = 3;

        public static final int BUCKET_BEAMBREAK = 1;
        public static final int INTAKE_BEAMBREAK = 2;
    }

    public static class PWM_IDS {
        public static final int SERVO = 9;
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
            public static final int CORAL_HOLDER_MC = 12;
            public static final int CORAL_INTAKE_MC = 13;
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
            public static final int ELEVATOR_MASTER = 18; // Right
            public static final int ELEVATOR_FOLLOWER = 19; // Left
        }
        
    }

    public static final class AutoConstants {
        public static final double AUTO_DRIVE_P = 6;
        public static final double AUTO_DRIVE_I = 0;
        public static final double AUTO_DRIVE_D = 0;
        public static final PIDConstants AUTO_DRIVE_PID = new PIDConstants(
            AUTO_DRIVE_D,
            AUTO_DRIVE_I,
            AUTO_DRIVE_D);
  
    }    
    public static class VisionConstants {
        public static final String CORAL_CAM_NAME = "coral";
        public static final String REEF_CAM_NAME = "reef";
        public static final Transform3d CORAL_CAM_TRANSFORM = new Transform3d(null, null, null, null); // find
        // Record actual transform in a comment here (readable by a human)
        public static final Transform3d REEF_CAM_TRANSFORM = new Transform3d(null, null, null, null); // find
        // Record actual transform in a comment here (readable by a human)
        public static final Transform3d LIMELIGHT_TRANSFORM = new Transform3d(null, null, null, null);//Find later
      }
    
    public static class FieldConstants {

        // tune this
        public static final double FIELD_WIDTH = 16.541;
        public static final double FIELD_LENGTH = 8.211;
        public static final double REEF_APRILTAG_HEIGHT = 6.875; // feet? figure out units
        public static final double PROCCESSOR_APRILTAG_HEIGHT = 45.875;
        public static final double CORAL_APRILTAG_HEIGHT = 53.25;
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

        public static final int[] RED_CORAL_TAGS = {6, 7, 8, 9, 10, 11};
        public static final int[] BLUE_CORAL_TAGS = {17, 18, 19, 20, 21, 22};
        public static final int[] RED_FEEDER_TAGS = {1,2};
        public static final int[] BLUE_FEEDER_TAGS = {12,13};
        public static final int[] RED_SIDE_CLIMB_TAGS = {4,5};
        public static final int[] BLUE_SIDE_CLIMB_TAGS = {14,15};
        public static final int[] PROCESSOR_TAGS = {1,15};
    }

}
