package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.shuffleboard.LayoutType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.AutoWaypoints.AutoPoses;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class CompetitionLogger implements Loggable {
    private static CompetitionLogger SINGLE_INSTANCE = new CompetitionLogger();

    public static CompetitionLogger getInstance() {
        return SINGLE_INSTANCE;
    }

    public boolean beginClimb = false;


    
    private SwerveDriveDB mySwerveDrivebuttons = new SwerveDriveDB();
   // private String autoChooser;
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

    }

    @Log
    public float getPitch(){
      return SwerveMap.GYRO.getPitch();
    }

}