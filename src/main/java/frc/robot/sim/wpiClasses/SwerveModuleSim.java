package frc.robot.sim.wpiClasses;

import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.swerve.SwerveConstants;

public class SwerveModuleSim {

    public static final double kWheelDiameter = Units.inchesToMeters(4);
    public static final double kWheelCircumference = kWheelDiameter*Math.PI;
    // Simulation
    private final TalonFXSimCollection driveMotorSim;
    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    public static final SimpleMotorFeedforward kDriveFF = new SimpleMotorFeedforward( // real
    0.2, // Voltage to break static friction
    2.25, // Volts per meter per second
    0.17 // Volts per meter per second squared
    );
    // Steer feed forward
    public static final SimpleMotorFeedforward kSteerFF = new SimpleMotorFeedforward( // real
        0.55, // Voltage to break static friction
        0.23, // Volts per radian per second
        0.0056 // Volts per radian per second squared
    );
    private final FlywheelSim driveWheelSim = new FlywheelSim(
        LinearSystemId.identifyVelocitySystem(
            kDriveFF.kv * kWheelCircumference / (2*Math.PI),
            kDriveFF.ka * kWheelCircumference / (2*Math.PI)
        ),
        DCMotor.getFalcon500(1),
        SwerveConstants.DRIVE_MOTOR_GEARING
    );
    private final TalonFXSimCollection steerMotorSim;
    private final FlywheelSim steeringSim = new FlywheelSim(
        LinearSystemId.identifyVelocitySystem(kSteerFF.kv, kSteerFF.ka),
        DCMotor.getFalcon500(1),
        SwerveConstants.STEERING_MOTOR_GEARING
    );
    public SwerveModuleSim(TalonFX driveMotor, TalonFX steerMotor) {
        this.driveMotor= driveMotor;
        this.steerMotor= steerMotor;

        driveMotorSim = driveMotor.getSimCollection();
        steerMotorSim = steerMotor.getSimCollection();
    }

    public void simulationPeriodic(){
        // apply our commanded voltage to our simulated physics mechanisms
        double driveVoltage = driveMotorSim.getMotorOutputLeadVoltage();
        if(driveVoltage >= 0) driveVoltage = Math.max(0, driveVoltage-kSteerFF.ks);
        else driveVoltage = Math.min(0, driveVoltage+kSteerFF.ks);
        driveWheelSim.setInputVoltage(driveVoltage);

        double steerVoltage = steerMotorSim.getMotorOutputLeadVoltage();
        if(steerVoltage >= 0) steerVoltage = Math.max(0, steerVoltage-kSteerFF.ks);
        else steerVoltage = Math.min(0, steerVoltage+kSteerFF.ks);
        steeringSim.setInputVoltage(steerVoltage);
        
        driveWheelSim.update(0.02);
        steeringSim.update(0.02);

        // update our simulated devices with our simulated physics results
        double driveMotorVelocityNative = rotationsToVelocity(driveWheelSim.getAngularVelocityRPM()/60, SwerveConstants.DRIVE_MOTOR_GEARING);
        double driveMotorPositionDeltaNative = driveMotorVelocityNative*10*0.02;
        driveMotorSim.setIntegratedSensorVelocity((int)driveMotorVelocityNative);
        driveMotorSim.addIntegratedSensorPosition((int)(driveMotorPositionDeltaNative));
        driveMotorSim.setSupplyCurrent(driveWheelSim.getCurrentDrawAmps()/2);

        //SmartDashboard.putNumber("Steer Sim Model Velocity", steeringSim.getAngularVelocityRPM());
        double steerMotorVelocityNative = rotationsToVelocity(steeringSim.getAngularVelocityRPM()/60, SwerveConstants.STEERING_MOTOR_GEARING);
        double steerMotorPositionDeltaNative = steerMotorVelocityNative*10*0.02;
        steerMotorSim.setIntegratedSensorVelocity((int)steerMotorVelocityNative);
        steerMotorSim.addIntegratedSensorPosition((int)(steerMotorPositionDeltaNative));
        steerMotorSim.setSupplyCurrent(steeringSim.getCurrentDrawAmps()/2);
        
        //steerEncoderSim.setVelocity((int)(rotationsToVelocity(steeringSim.getAngularVelocityRPM()/60, 1)*2));
        //steerEncoderSim.setRawPosition((int)(getIntegratedHeading().getDegrees()/360.0*4096));

        driveMotorSim.setBusVoltage(RobotController.getBatteryVoltage());
        steerMotorSim.setBusVoltage(RobotController.getBatteryVoltage());
        //steerEncoderSim.setBusVoltage(RobotController.getBatteryVoltage());
    }

    public double getDriveCurrentDraw(){
        return driveMotor.getSupplyCurrent();
    }
    public double getSteerCurrentDraw(){
        return steerMotor.getSupplyCurrent();
    }

    public static double positionToRotations(double nativePosition, double motorRotationsPerMechanismRotation){
        return nativePosition / 2048 / motorRotationsPerMechanismRotation;
    }
    public static double positionToDegrees(double nativePosition, double motorRotationsPerMechanismRotation){
        return positionToRotations(nativePosition, motorRotationsPerMechanismRotation) * 360;
    }
    public static double positionToRadians(double nativePosition, double motorRotationsPerMechanismRotation){
        return positionToRotations(nativePosition, motorRotationsPerMechanismRotation) * 2 * Math.PI;
    }
    public static double positionToMeters(double nativePosition, double motorRotationsPerMechanismRotation, double circumferenceMeters){
        return positionToRotations(nativePosition, motorRotationsPerMechanismRotation) * circumferenceMeters;
    }
    public static double rotationsToPosition(double rotations, double motorRotationsPerMechanismRotation){
        return rotations * 2048 * motorRotationsPerMechanismRotation;
    }
    public static double degreesToPosition(double degrees, double motorRotationsPerMechanismRotation){
        return rotationsToPosition(degrees/(360), motorRotationsPerMechanismRotation);
    }
    public static double radiansToPosition(double radians, double motorRotationsPerMechanismRotation){
        return rotationsToPosition(radians/(2*Math.PI), motorRotationsPerMechanismRotation);
    }
    public static double metersToPosition(double meters, double motorRotationsPerMechanismRotation, double circumferenceMeters){
        return rotationsToPosition(meters/(circumferenceMeters), motorRotationsPerMechanismRotation);
    }

    public static double velocityToRotations(double nativeVelocity, double motorRotationsPerMechanismRotation){
        return nativeVelocity * 10 / 2048 / motorRotationsPerMechanismRotation;
    }
    public static double velocityToDegrees(double nativeVelocity, double motorRotationsPerMechanismRotation){
        return velocityToRotations(nativeVelocity, motorRotationsPerMechanismRotation) * 360;
    }
    public static double velocityToRadians(double nativeVelocity, double motorRotationsPerMechanismRotation){
        return velocityToRotations(nativeVelocity, motorRotationsPerMechanismRotation) * 2 * Math.PI;
    }
    public static double velocityToMeters(double nativeVelocity, double motorRotationsPerMechanismRotation, double circumferenceMeters){
        return velocityToRotations(nativeVelocity, motorRotationsPerMechanismRotation) * circumferenceMeters;
    }
    public static double rotationsToVelocity(double rotations, double motorRotationsPerMechanismRotation){
        return rotations * 2048 / 10 * motorRotationsPerMechanismRotation;
    }
    public static double degreesToVelocity(double degrees, double motorRotationsPerMechanismRotation){
        return rotationsToVelocity(degrees/(360), motorRotationsPerMechanismRotation);
    }
    public static double radiansToVelocity(double radians, double motorRotationsPerMechanismRotation){
        return rotationsToVelocity(radians/(2*Math.PI), motorRotationsPerMechanismRotation);
    }
    public static double metersToVelocity(double meters, double motorRotationsPerMechanismRotation, double circumferenceMeters){
        return rotationsToVelocity(meters/(circumferenceMeters), motorRotationsPerMechanismRotation);
    }


}
