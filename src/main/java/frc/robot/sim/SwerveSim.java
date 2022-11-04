package frc.robot.sim;

import java.util.List;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.sim.wpiClasses.QuadSwerveSim;
import frc.robot.sim.wpiClasses.SwerveModuleSim;
import frc.robot.swerve.SwerveConstants;

public class SwerveSim {
    private static SwerveSim SINGLE_INSTANCE = new SwerveSim();

    public static SwerveSim getInstance() {
        return SINGLE_INSTANCE;
    }

    public List<SwerveModuleSim> swerveModuleSimList = List.of(new SwerveModuleSim(DCMotor.getFalcon500(1), 
    DCMotor.getFalcon500(1), 
    SwerveConstants.WHEEL_RADIUS_METERS,
    SwerveConstants.STEERING_MOTOR_GEARING, // steering motor rotations per wheel steer rotation
    SwerveConstants.DRIVE_MOTOR_GEARING,
    SwerveConstants.STEERING_MOTOR_GEARING, // same as motor rotations because NEO encoder is on motor shaft
    SwerveConstants.DRIVE_MOTOR_GEARING,
    1.3,
    0.7,
    56.6 * 9.81 / QuadSwerveSim.NUM_MODULES, 
    0.01 
    ), new SwerveModuleSim(DCMotor.getFalcon500(1), 
    DCMotor.getFalcon500(1), 
    SwerveConstants.WHEEL_RADIUS_METERS,
    SwerveConstants.STEERING_MOTOR_GEARING, // steering motor rotations per wheel steer rotation
    SwerveConstants.DRIVE_MOTOR_GEARING,
    SwerveConstants.STEERING_MOTOR_GEARING, // same as motor rotations because NEO encoder is on motor shaft
    SwerveConstants.DRIVE_MOTOR_GEARING,
    1.3,
    0.7,
    56.6 * 9.81 / QuadSwerveSim.NUM_MODULES, 
    0.01 
    ), new SwerveModuleSim(DCMotor.getFalcon500(1), 
    DCMotor.getFalcon500(1), 
    SwerveConstants.WHEEL_RADIUS_METERS,
    SwerveConstants.STEERING_MOTOR_GEARING, // steering motor rotations per wheel steer rotation
    SwerveConstants.DRIVE_MOTOR_GEARING,
    SwerveConstants.STEERING_MOTOR_GEARING, // same as motor rotations because NEO encoder is on motor shaft
    SwerveConstants.DRIVE_MOTOR_GEARING,
    1.3,
    0.7,
    56.6 * 9.81 / QuadSwerveSim.NUM_MODULES, 
    0.01 
    ), new SwerveModuleSim(DCMotor.getFalcon500(1), 
    DCMotor.getFalcon500(1), 
    SwerveConstants.WHEEL_RADIUS_METERS,
    SwerveConstants.STEERING_MOTOR_GEARING, // steering motor rotations per wheel steer rotation
    SwerveConstants.DRIVE_MOTOR_GEARING,
    SwerveConstants.STEERING_MOTOR_GEARING, // same as motor rotations because NEO encoder is on motor shaft
    SwerveConstants.DRIVE_MOTOR_GEARING,
    1.3,
    0.7,
    56.6 * 9.81 / QuadSwerveSim.NUM_MODULES, 
    0.01 
    ));
  public QuadSwerveSim simSwerve = new QuadSwerveSim(SwerveConstants.TRACK_WIDE, 
    SwerveConstants.WHEEL_BASE_METERS, 
    56.6, 1.0/12.0 * 56.6 * Math.pow((SwerveConstants.TRACK_WIDE*1.1),2) * 2, swerveModuleSimList); 
}
