package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MagicManager extends SubsystemBase {
    int level;
    boolean enableMagic;
    

    public MagicManager() {
        level = 1;
        enableMagic = false;
    }

    public int getLevel() {
        return level;
    }

    public boolean getMagic() {
        return enableMagic;
    }

    public void setLevel(int lev) {
        level = lev;
    }

    public void setMagic(boolean sm) {
        enableMagic = sm;
    }

    public Command setLevelCommand(int lev) {
        return runOnce(
            () -> level = lev
        );
    }

    public Command setMagicCommand() {
        return runOnce(
            () -> {
                enableMagic = true;
            }
        );
    }

    public Command turnOffMagicCommand() {
        return runOnce(
            () -> {
                enableMagic = false;
            }
        );
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Current Magic Level", level);
        SmartDashboard.putBoolean("Magic Enabled", enableMagic);
    }


}
