package frc.robot;

import javax.print.attribute.standard.Compression;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class CompetitionLogger implements Loggable {
    private static CompetitionLogger SINGLE_INSTANCE = new CompetitionLogger();

    public static CompetitionLogger getInstance() {
        return SINGLE_INSTANCE;
    }

    public boolean beginClimb = false;
    public Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);


    //KEEP THIS INSTANTIATION IT'S FOR LOGGING
    private SwerveDriveDB mySwerveDrivebuttons = new SwerveDriveDB();
   
    public class SwerveDriveDB implements Loggable {
        
        
        @Override
        public int[] configureLayoutSize() {
            int[] size = {2,3};
            return size;
        }
        @Config
        public void HoldRobotAngle(boolean _input){
            Robot.SWERVEDRIVE.holdRobotAngleEnabled = _input;
        }
        @Config
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

        @Config
        public void resetDisplacement(boolean _input){
            SwerveMap.GYRO.resetDisplacement();
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
    public float navXData() {
        return SwerveMap.GYRO.getDisplacementX();
    }

    @Log
    public float navYData() {
        return SwerveMap.GYRO.getDisplacementY();
    }

    @Log
    public double getPressure() {
        return compressor.getPressure();
    }

    @Log
    public Field2d field2022(){
        return Robot.SWERVEDRIVE.field;
    }


}