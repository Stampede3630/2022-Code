package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;

public class SwerveCharacterization implements Loggable{
    private static SwerveCharacterization SINGLE_INSTANCE = new SwerveCharacterization();
    private final NetworkTableEntry autoSpeedEntry = NetworkTableInstance.getDefault().getEntry("/robot/autospeed");
    private final NetworkTableEntry telemetryEntry = NetworkTableInstance.getDefault().getEntry("/robot/telemetry");
    //private final NetworkTableEntry rotateEntry = NetworkTableInstance.getDefault().getEntry("/robot/rotate");
    //private final NetworkTableEntry autoSpeedEntry = NetworkTableInstance.getDefault().getEntry("/SmartDashboard/SysIdAutoSpeed");
    //private final NetworkTableEntry telemetryEntry = NetworkTableInstance.getDefault().getEntry("/SmartDashboard/SysIdTelemetry");
    
    String data = "";
    int counter = 0;
    double startTime = 0;
    double[] numberArray = new double[10];
    ArrayList<Double> entries = new ArrayList<Double>();

    //private List<Double> telemetryData = new ArrayList<>();

    private double priorAutospeed = 0.0;
    public static SwerveCharacterization getInstance() {
        return SINGLE_INSTANCE;
    }

    @Config.ToggleSwitch(name="reset char?", defaultValue = false)
    public void init(boolean _input) {
        if (_input){
        NetworkTableInstance.getDefault().setUpdateRate(10.0e-3);
        counter = 0;
        _input = false;
        }
    }
    
    public void periodic() {
        double now = Timer.getFPGATimestamp();
        double position = Robot.SWERVEDRIVE.m_odometry.getPoseMeters().getX()/Constants.METERSperWHEEL_REVOLUTION;
        System.out.println(position);
        double velocity = (Robot.SWERVEDRIVE.getBackLeftSpeed()+
        Robot.SWERVEDRIVE.getFrontLeftSpeed()+
        Robot.SWERVEDRIVE.getBackRightSpeed()+
        Robot.SWERVEDRIVE.getFrontRightSpeed())
                / 4 / Constants.METERSperWHEEL_REVOLUTION;

        double battery = RobotController.getBatteryVoltage();
        double motorVoltage = battery * Math.abs(priorAutospeed);

        double autospeed = autoSpeedEntry.getDouble(0.0);
        priorAutospeed = autospeed;

        Robot.SWERVEDRIVE.setSDxSpeed(autospeed*Constants.MAX_SPEED_METERSperSECOND);

        numberArray[0] = now;
        numberArray[1] = battery;
        numberArray[2] = autospeed;
        numberArray[3] = motorVoltage;
        numberArray[4] = motorVoltage;
        numberArray[5] = position;
        numberArray[6] = position;
        numberArray[7] = velocity;
        numberArray[8] = velocity;
        numberArray[9] = Robot.SWERVEDRIVE.m_odometry.getPoseMeters().getRotation().getRadians();
        for (double num : numberArray) {
            entries.add(num);
          }
          counter++;
    }

    public void disabled(boolean interrupted) {
        double elapsedTime = Timer.getFPGATimestamp() - startTime;
        System.out.println("Robot disabled");
        // data processing step
        data = entries.toString();
        data = data.substring(1, data.length() - 1) + ", ";
        telemetryEntry.setString(data);
        entries.clear();
        System.out.println("Robot disabled");
        System.out.println("Collected : " + counter + " in " + elapsedTime + " seconds");
        data = "";
    }
}
