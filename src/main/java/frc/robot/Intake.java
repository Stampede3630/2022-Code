package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class Intake implements Loggable {
  
  private static Intake SINGLE_INSTANCE = new Intake();
  
  private static WPI_TalonFX intakeDrive;
  private static WPI_TalonFX indexBottom;
  private static WPI_TalonFX indexTop;
  private static DoubleSolenoid intakeSolenoid;
  // SWITCHES: GREEN = NOT PRESSED, RED = PRESSED
  // SWITCHES RETURN TRUE WHEN NOT PRESSED, FALSE WHEN PRESSED
  private static DigitalInput bottomLimitSwitch;
  private static DigitalInput topLimitSwitch;
  private static boolean cargoInTransit = false;

  public static Intake getInstance() {
      return SINGLE_INSTANCE;
  }

  public void init(){
    intakeDrive  = new WPI_TalonFX(13);
    indexBottom = new WPI_TalonFX(12);
    indexTop = new WPI_TalonFX(16);

    intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 5, 6);

    bottomLimitSwitch = new DigitalInput(1);
    topLimitSwitch = new DigitalInput(0);
  }
  public void intakePeriodic(){
    intake();
    shootIndexManager();
  }

  private void intake() {
    if (Robot.xbox.getRightTriggerAxis() > 0) {   //right trigger held --> intake goes down and spins intake motor
      intakeSolenoid.set(Value.kReverse);
      intakeDrive.set(ControlMode.PercentOutput, .3);

    } else { 
      intakeSolenoid.set(Value.kForward); //pulls intake back up and stops spinning
      intakeDrive.set(ControlMode.PercentOutput, 0);
    }
  }

  private void shootIndexManager() {
    if (Robot.SHOOTER.shooterAtSpeed() && Robot.xbox.getLeftTriggerAxis() > 0) {  //once shooter gets up to speed AND left trigger held, balls fed to shooter
      if (!bottomLimitSwitch.get() && !topLimitSwitch.get()) {
        indexTop.set(ControlMode.PercentOutput, 0.2); //if there's only one ball being shot
      } else {
        indexTop.set(ControlMode.PercentOutput, 0.2); //if two balls are being shot
        indexBottom.set(ControlMode.PercentOutput, 0.2);
      }
    } else {
      indexerDrive(); //defaults to state machine below
    }
  }

  private void indexerDrive() {
      switch (indexManager()) {
        case "1 Ball": //hold the ball at the top of tower
          indexBottom.set(ControlMode.PercentOutput, 0.1);
          indexTop.set(ControlMode.PercentOutput, 0);
          // shootIndexManager();
          break;

        case "2 Balls": //indexer full
          indexBottom.set(ControlMode.PercentOutput, 0); 
          indexTop.set(ControlMode.PercentOutput, 0);
          // shootIndexManager();
          break;

        case "Cargo in Transit":  //bring ball from intake to top of tower
          indexTop.set(ControlMode.PercentOutput, 0.2);
          indexBottom.set(ControlMode.PercentOutput, 0.2);
          break;

        case "Reverse Intake":  //both intakes go backwards
          indexTop.set(ControlMode.PercentOutput, -0.2);
          indexBottom.set(ControlMode.PercentOutput, -0.2);
          break;

        case "Intake Ball": //spins bottom intake while ball in being intaked
          indexBottom.set(ControlMode.PercentOutput, 0.2);
          break;

        default:  //everything stops
          indexBottom.set(ControlMode.PercentOutput, 0);
          indexTop.set(ControlMode.PercentOutput, 0);
          break;
      }
    } 

  private String indexManager() {
    if (Robot.xbox.getBButton()) {
      return "Reverse Intake";
    } else if (!bottomLimitSwitch.get() && !topLimitSwitch.get()) { //both switches pressed
      return "2 Balls";
    } else if (!bottomLimitSwitch.get() || (cargoInTransit && bottomLimitSwitch.get() && topLimitSwitch.get())) { //bottom switch pressed/cargo in transit
      cargoInTransit = true;
      return "Cargo in Transit";
    } else if (!topLimitSwitch.get()) { //top switch pressed
      cargoInTransit = false;
      return "1 Ball";
    } else if (bottomLimitSwitch.get() && Robot.xbox.getRightTriggerAxis() > 0) { //right trigger held AND nothing pressing the bottom switch
      return "Intake Ball";
    } else {
      return "default";
    }
  }

  @Log.BooleanBox(rowIndex = 1, columnIndex = 2)
  public boolean getBottomLimitSwitch() {
    return bottomLimitSwitch.get();
  }

  @Log.BooleanBox(rowIndex = 3, columnIndex = 4)
  public boolean getTopLimitSwitch() {
    return topLimitSwitch.get();
  }
}
