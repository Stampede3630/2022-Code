package frc.robot;

import javax.print.attribute.standard.Compression;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import io.github.oblarg.oblog.annotations.Log.BooleanBox;

public class CompetitionLogger implements Loggable {
    private static CompetitionLogger SINGLE_INSTANCE = new CompetitionLogger();

    public static CompetitionLogger getInstance() {
        return SINGLE_INSTANCE;
    }

    @Log
    double shooterAngle = Robot.SHOOTER.calculateShooterAngle();

    public boolean beginClimb = false;
    public Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);

    //KEEP THIS INSTANTIATION IT'S FOR LOGGING
    private SwerveDriveDB mySwerveDrivebuttons = new SwerveDriveDB();
    
    public class SwerveDriveDB implements Loggable {
        @Override
        public int[] configureLayoutSize() {
            int[] size = {2,4};
            return size;
        }
        @Override
        public int[] configureLayoutPosition() {
            int[] position = {0,0};
            return position;
        }
        @Config 
        public void HoldRobotAngle(boolean _input){
            Robot.SWERVEDRIVE.holdRobotAngleEnabled = _input;
        }
        @Config(defaultValueBoolean = true)
        public void FieldRelativeDrive(boolean _input){
            Robot.SWERVEDRIVE.SDFieldRelative = _input;
        }
        @Config
        public void ResetGyroAndOdometry(boolean _input){
            Robot.SWERVEDRIVE.resetGyroAndOdometry(_input);
        }

        @Config
        private void beginClimbing(boolean _input){
            if(_input){
                beginClimb = true;
            }
        }
        
        @Config (defaultValueBoolean = false)
        public void setFancyShot(boolean _input){
            Robot.SHOOTER.fancyShot = _input;
        }

        @Config (defaultValueBoolean = true)
        public void limelightAim(boolean _input){
                Robot.SHOOTER.limelightShooting = _input;
        }
        
        
        @Log
        public boolean allZeroedModules(){
            return SwerveMap.BackLeftSwerveModule.hasSwerveZeroingOccurred &&
            SwerveMap.BackRightSwerveModule.hasSwerveZeroingOccurred &&
            SwerveMap.FrontLeftSwerveModule.hasSwerveZeroingOccurred &&
            SwerveMap.FrontRightSwerveModule.hasSwerveZeroingOccurred;
        }
    }
    
    // @Log
    // public float getPitch(){
    //     return SwerveMap.GYRO.getPitch();
    // }
    
    // @Log
    // public float navXData() {
    //     return SwerveMap.GYRO.getDisplacementX();
    // }
    
    // @Log
    // public float navYData() {
    //     return SwerveMap.GYRO.getDisplacementY();
    // }
    
    @Log.NumberBar(min = 0, max = 140, rowIndex = 2, columnIndex = 2)
    public double getPressure() {
        return compressor.getPressure();
    }
    
    // @Log
    // public Field2d field2022(){
    //     return Robot.SWERVEDRIVE.field;
    // }
    
    @Log(rowIndex = 3, columnIndex = 2)
    public double getMatchTimer() {
        return Timer.getMatchTime();
    }

    // @Log
    // public double leftJoystickValues() {
    //     return Robot.xbox.getLeftX();
    // }

    // @Log
    // public double rightJoystickValues() {
    //     return Robot.xbox.getRightX();
    // }

    @Log.NumberBar (min = 0, max = 14, rowIndex = 1, columnIndex = 2)
    public double batteryVoltage() {
        return RobotController.getBatteryVoltage();
    }
    
}