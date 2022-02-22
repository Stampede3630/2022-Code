package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;

public class CompetitionLogger implements Loggable {

    public void competitionTab(boolean HoldRobotAngle, boolean FieldOriented, boolean ResetGyroAndOdometry) {
        Robot.SWERVEDRIVE.holdRobotAngleEnabled = HoldRobotAngle;
        Robot.SWERVEDRIVE.SDFieldRelative = FieldOriented;
        Robot.SWERVEDRIVE.resetGyroAndOdometry(ResetGyroAndOdometry);
    }
    
}