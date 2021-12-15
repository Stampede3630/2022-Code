package frc.robot;

public class Constants{
    
    public static final int FRDriveID = 6;
    public static final int FLDriveID = 12;
    public static final int BRDriveID = 8;
    public static final int BLDriveID = 10;

    public static final int FRSteerID = 7;
    public static final int FLSteerID = 5;
    public static final int BRSteerID = 9;
    public static final int BLSteerID = 11;

    public static final int FRSensorID = 3;
    public static final int FLSensorID = 4;
    public static final int BRSensorID = 2;
    public static final int BLSensorID = 1;

    public static final Gains FRGains = new Gains(1.1, .6, 0);
    public static final Gains FLGains = new Gains(1.1, .6, 0);
    public static final Gains BRGains = new Gains(1.1, .6, 0);
    public static final Gains BLGains = new Gains(1.1, .6, 0);

    public static final int kDefaultPIDSlotID = 0;
    public static final int kDefaultTimeout = 30;//milliseconds
    public static final int kDefaultClosedLoopError = 10;//sensor units

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
    }
}