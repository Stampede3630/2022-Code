package frc.robot;

import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;

public class AutoContainer implements Loggable {
    
    private static AutoContainer SINGLE_INSTANCE = new AutoContainer();

    public static AutoContainer getInstance() {
        return SINGLE_INSTANCE;
    }

    @Config.ToggleButton(name="Field Position 1", defaultValue = false, rowIndex = 1, columnIndex = 1, height = 1, width = 2)
    private static void fieldPosition1(boolean pos1) {
        if (pos1) { setAuto(1); }
        else { setAuto(5); }
    }

    @Config.ToggleButton(name="Field Position 2", defaultValue = false, rowIndex = 1, columnIndex = 3, height = 1, width = 2)
    private static void fieldPosition2(boolean pos2) {
        if (pos2) { setAuto(2); }
        else { setAuto(6); }
    }

    @Config.ToggleButton(name="Field Position 3", defaultValue = false, rowIndex = 1, columnIndex = 5, height = 1, width = 2)
    private static void fieldPosition3(boolean pos3) {
        if (pos3) { setAuto(3); }
        else { setAuto(7); }
    }

    @Config.ToggleButton(name="Field Position 4", defaultValue = false, rowIndex = 1, columnIndex = 7, height = 1, width = 2)
    private static void fieldPosition4(boolean pos4) {
        if (pos4) { setAuto(4); }
        else {setAuto(8); }
    }

    private static void setAuto(int autoPosition) {
        
        switch (autoPosition) { // These are fake values, put in real stuff
            case 1:
                SwerveMap.GYRO.setAngleAdjustment(45);
                break;
            case 2:
                SwerveMap.GYRO.setAngleAdjustment(90);
                break;
            case 3:
                SwerveMap.GYRO.setAngleAdjustment(180);
                break;
            case 4:
                SwerveMap.GYRO.setAngleAdjustment(270);
                break;
            case 5: SwerveMap.GYRO.setAngleAdjustment(-45);
                break;
            case 6: SwerveMap.GYRO.setAngleAdjustment(-90);
                break;
            case 7: SwerveMap.GYRO.setAngleAdjustment(-180);
                break;
            case 8: SwerveMap.GYRO.setAngleAdjustment(-270);
            default:
                SwerveMap.GYRO.setAngleAdjustment(0);
                break;
        }
    }
}
