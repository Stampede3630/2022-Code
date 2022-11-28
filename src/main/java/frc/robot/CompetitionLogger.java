package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.robot.swerve.SwerveMap;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;


public class CompetitionLogger implements Loggable {
    private static CompetitionLogger SINGLE_INSTANCE = new CompetitionLogger();

    public static CompetitionLogger getInstance() {
        return SINGLE_INSTANCE;
    }

    @Log
    double shooterAngle = Robot.SHOOTER.calculateShooterAngle();

    public Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);
    public PowerDistribution myPD = new PowerDistribution(36, ModuleType.kRev);

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
        // @Config 
        // public void HoldRobotAngle(boolean _input){
        //     Robot.SWERVEDRIVE.holdRobotAngleEnabled = _input;
        // }
        @Config(defaultValueBoolean = true)
        public void FieldRelativeDrive(boolean _input){
            Robot.SWERVEDRIVE.SDFieldRelative = _input;
        }

        @Config(defaultValueBoolean = false)
        public void SetShotBlock(boolean _input){
            Robot.SHOOTER.shotBlock = _input;
        }
        
        @Config (defaultValueBoolean = true)
        public void setFancyShot(boolean _input){
            Robot.SHOOTER.fancyShot = _input;
        }

        @Config (defaultValueBoolean = true)
        public void limelightAim(boolean _input){
                Robot.SHOOTER.limelightShooting = _input;
        }

        @Log
        public double LLDistance() {
            return Robot.SHOOTER.distance;
        }
        
        
        @Log
        public boolean allZeroedModules(){
            return SwerveMap.BackLeftSwerveModule.hasSwerveZeroingOccurred &&
            SwerveMap.BackRightSwerveModule.hasSwerveZeroingOccurred &&
            SwerveMap.FrontLeftSwerveModule.hasSwerveZeroingOccurred &&
            SwerveMap.FrontRightSwerveModule.hasSwerveZeroingOccurred;
        }
    }
    
    @Log
    public float getPitch(){
        return SwerveMap.GYRO.getPitch();
    }

    @Log 
    public float getPitchV() {
        return SwerveMap.GYRO.getVelocityY();
    }
    
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

    @Config(name = "kill battery?", defaultValueBoolean = false, rowIndex = 2, columnIndex = 4, height = 1, width = 1)
    public void killTheBattery(boolean _input) {
        Robot.SHOOTER.homocideTheBattery = _input;
    }

    @Config(name = "funnie clime time", defaultValueBoolean = false, rowIndex = 3, columnIndex = 5, height = 1, width = 1)
    public void isItDDRTime(boolean _input){
        Robot.CLIMBER.ddrTime = _input;
    }

    
    // @Log
    // public Field2d field2022(){
    //     return Robot.SWERVEDRIVE.field;
    // }
    
    @Log(rowIndex = 3, columnIndex = 2)
    public double getMatchTimer() {
        return Timer.getMatchTime();
    }

    // @Log.BooleanBox
    // public boolean getBottomLimit() {
    //     return Robot.INTAKE.bottomLimitSwitch.get();
    // }

    // @Log
    // public double leftJoystickValues() {
    //     return Robot.xbox.getLeftX();
    // }

    // @Log
    // public double rightJoystickValues() {
    //     return Robot.xbox.getRightX();
    // }

    @Log (rowIndex = 3, columnIndex = 7)
    public double getMyPD() {
        return Robot.myWattThingy;
    }

    @Log.NumberBar (min = 0, max = 14, rowIndex = 1, columnIndex = 2)
    public double batteryVoltage() {
        return RobotController.getBatteryVoltage();
    } 


}
       