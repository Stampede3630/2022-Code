package frc.robot;

public class Constants{
    public static final double MAX_SPEED = 4.3;// 
    public static final double Converted_MAX_SPEED = 12676;//4.3/10/(.051*2*Math.PI*9200);
    
    public static final double WHEEL_RADIUS = .051;
    
    //Swerve Drive Motor IDs
    public static final int FRDriveID = 6;
    public static final int FLDriveID = 12;
    public static final int BRDriveID = 8;
    public static final int BLDriveID = 10;

    //Swerve Steer Motor IDs
    public static final int FRSteerID = 7;
    public static final int FLSteerID = 5;
    public static final int BRSteerID = 9;
    public static final int BLSteerID = 11;

    //Swerve CANCoder Sensor IDs
    public static final int FRSensorID = 3;
    public static final int FLSensorID = 4;
    public static final int BRSensorID = 2;
    public static final int BLSensorID = 1;

    //Swerve CANCoder offsets
    //CHANGE TO 0 first, reset the sensor, zero out the motor and place the OPPOSITE of the value
    public static double FRSensorOffset = -48.955;
    public static double FLSensorOffset = 52.119;
    public static double BRSensorOffset = 175.518;
    public static double BLSensorOffset = -34.453;

    //Swerve Steering PIDs (kP, kI, kD)
    public static Gains FRSteerGains = new Gains(1.2, 0, 0);
    public static Gains FLSteerGains = new Gains(1.2, 0, 0);
    public static Gains BRSteerGains = new Gains(1.2, 0, 0);
    public static Gains BLSteerGains = new Gains(1.2, 0, 0);

    //Swerve Driving PIDs (kP, kI, kD)
    public static Gains FRDriveGains = new Gains(0.1, 0.001, 5, 1023.0/Constants.Converted_MAX_SPEED);
    public static Gains FLDriveGains = new Gains(0.1, 0.001, 5, 1023.0/Constants.Converted_MAX_SPEED);
    public static Gains BRDriveGains = new Gains(0.1, 0.001, 5, 1023.0/Constants.Converted_MAX_SPEED);
    public static Gains BLDriveGains = new Gains(0.1, 0.001, 5, 1023.0/Constants.Converted_MAX_SPEED);

    //CTRE CAN-based constants
    public static final int kDefaultPIDSlotID = 0;
    public static final int kDefaultTimeout = 30;//milliseconds
    public static final int kDefaultClosedLoopError = 5;//sensor units

    public static enum inputmode {TELEOP, DEBUG, AUTO};

    public static class Gains {
        public final double kP;
        public final double kI;
        public final double kD;
        public final double kF;
        public final int kIzone;
        public final double kPeakOutput;
        /**
         * @param _kP
         * @param _kI
         * @param _kD
         */
        public Gains(double _kP, double _kI, double _kD){
            kP = _kP;
            kI = _kI;
            kD = _kD;
            kF = 0;
            kIzone = 0;
            kPeakOutput = 1;
        }
        public Gains(double _kP, double _kD, double _kI, double _kF){
            kP = _kP;
            kI = _kI;
            kD = _kD;
            kF = _kF;
            kIzone = 300;
            kPeakOutput = 1;
        }
    }
}