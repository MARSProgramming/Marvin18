package frc.robot.constants;

import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map.Entry;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;

public final class DynamicConstants {
    /*
     * Add static classes below for Shuffleboard dynamic contants. Each class
     * represents
     * a subsystem and will have its own tab. These constants are stored on the RIO
     * through
     * NetworkTables json and should be saved to the driver station computer after
     * changes.
     * 
     * WARNING: Dynamic constants from the RIO may be read after initialization.
     * Ensure functionality
     * of any constants that are only read on startup.
     */

    public static class ElevatorSetpoints {
        public static double elevMax = 7.8;
        public static double elevL4 = 7.8;
        public static double elevL3 = 4.5;
        public static double elevL2 = 2.02;
        public static double elevL1 = 0;
        public static double elevAlgaeTop = 7.4;
        public static double elevAlgaeBot = 5.35;
        public static double elevAlgaeTee = 2.2;
        public static double elevAlgaeGround = 1;
        public static double elevClimb = 0;
        public static double elevClimbVoltage = -5;

        public static double elevTestPos = 1;
    }

    public static class AutoTagSetpoints {
        public static double NTALFwd_Bkwd = .45;
        public static double NTALLeft_Right = .08;

        public static double NTACFwd_Bkwd = 1;
        public static double NTACLeft_Right = 0;

        public static double NTARFwd_Bkwd = .45;
        public static double NTARLeft_Right = .45;

        public static double AutoTagSetpointsDegrees = 90;
    }

    public static class AlignTransforms {

        public static double RightXL1 = .41;
        public static double RightYL1 = .44;
        
        public static double RightXL2 = .41;
        public static double RightYL2 = .44;
        
        public static double RightXL3 = .41;
        public static double RightYL3 = .44;

        public static double RightXL4 = .45;
        public static double RightYL4 = .485;
        
        public static double RightRot = 90;
        
        public static double LeftXL1 = .41;
        public static double LeftYL1 = .10;
        
        public static double LeftXL2 = .41;
        public static double LeftYL2 = .10;
        
        public static double LeftXL3 = .41;
        public static double LeftYL3 = .10;
        
        public static double LeftXL4 = .45;
        public static double LeftYL4 = .13;


        public static double LeftRot = 90;
        
        public static double CentX = 1;
        public static double CentY = 0;
        public static double CentRot = 90;

        public static double AlgaeRot = 180;
        public static double feederX = .4;
        public static double feederY = 0.0;

    }

    public static class Drive {
        public static final double forwardBackward = 0.04;
        public static final double leftRight = 0.04;
    }

    public static class Algae {
        public static double intakePercent = .5;
        public static double outtakePercent = -.5;
    }

    public static class TestVoltages {
        public static double bucketTestOut = 0;
        public static double agitatorTestOut = 0;
        public static double algaeTestOut = 0;
    }

    public static class IRThresholds {
        public static double IRthreshold = 1.75;
    }

    private static HashMap<Field, SimpleWidget> entries;

    /*
     * Initializes all the Shuffleboard tabs and widgets by pulling their fields
     * from the provided classes
     */
    public static void init() {
        ShuffleboardTab subsystemIOTab;// = Shuffleboard.getTab("SubsystemIO");
        entries = new HashMap<>();

        // add all .class values of the static classes above
        Class<?>[] subsystems = { ElevatorSetpoints.class, TestVoltages.class, Algae.class, AutoTagSetpoints.class,
                AlignTransforms.class };

        for (Class<?> subsystem : subsystems) {
            Field[] fields = subsystem.getDeclaredFields();
            subsystemIOTab = Shuffleboard.getTab(subsystem.getSimpleName());
            for (Field field : fields) {
                if (java.lang.reflect.Modifier.isStatic(field.getModifiers()) && field.getType() == double.class) {
                    field.setAccessible(true);
                    double value = 0;
                    try {
                        value = field.getDouble(null);
                    } catch (IllegalAccessException a) {
                        System.out.println("Access Exception when reading subsystem IO");
                    }
                    entries.put(field,
                            subsystemIOTab
                                    .addPersistent(subsystem.getSimpleName() + ": " + field.getName(), "double", value)
                                    .withSize(2, 1));
                }
            }
        }
    }

    /*
     * Must be periodically called. Updates the constants values from the
     * Shuffleboard
     */
    public static void periodic() {
        for (Entry<Field, SimpleWidget> entry : entries.entrySet()) {
            try {
                entry.getKey().setDouble(null, entry.getValue().getEntry().getDouble(0));
            } catch (IllegalAccessException a) {
                System.out.println("Access Exception when writing subsystem IO");
            }
        }
    }
}
