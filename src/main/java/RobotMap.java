import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

public class RobotMap {
    public class Gains {
        public final double kP;
        public final double kI;
        public final double kD;
        public final double kF;
        public final int kIzone;
        public final double kPeakOutput;

        public Gains(double _kP, double _kI, double _kD, double _kF, int _kIzone, double _kPeakOutput){
            kP = _kP;
            kI = _kI;
            kD = _kD;
            kF = _kF;
            kIzone = _kIzone;
            kPeakOutput = _kPeakOutput;
        }
    }

    public class SteeringMotor{
        public final TalonFX kTalon;
        public final CANCoder kSensor;
        public final boolean kSensorInvert;
        
        final Gains kGAINS;

        public SteeringMotor (int _talonID, int _sensorID, boolean _sensorInvert, double _kP, double _kD, double _kI){
            kGAINS = new Gains(_kP, _kI, _kD, 0, 0, 1);
            kSensor = new CANCoder(_sensorID);
            kTalon = new TalonFX(_talonID);
            
            
        }
    }
}
