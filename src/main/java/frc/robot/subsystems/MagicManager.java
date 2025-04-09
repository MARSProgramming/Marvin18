package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MagicManager extends SubsystemBase {
    int level;
    

    public MagicManager() {

    }

    public int getLevel() {
        return level;
    }

    public void setLevel(int lev) {
        level = lev;
    }
}
