package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Intake implements Loggable {
  
  private static Intake SINGLE_INSTANCE = new Intake();
  
  private WPI_TalonFX intakeDrive;
  private WPI_TalonFX indexBottom;
  private WPI_TalonFX indexTop;
  private DoubleSolenoid intakeSolenoid;
  public DoubleSolenoid limelightSolenoid;
  // SWITCHES: GREEN = NOT PRESSED, RED = PRESSED
  // SWITCHES RETURN TRUE WHEN NOT PRESSED, FALSE WHEN PRESSED
  private DigitalInput bottomLimitSwitch;
  private DigitalInput topLimitSwitch;
  private boolean cargoInTransit = false;
  public boolean intakeNow = false;
  public boolean shootNow = false;
  public boolean limelightIsOpen = true; // rename and figure out if it starts open or closed

  public static Intake getInstance() {
    return SINGLE_INSTANCE;
  }

  public void init(){
    intakeDrive  = new WPI_TalonFX(13);
    indexBottom = new WPI_TalonFX(12);
    indexTop = new WPI_TalonFX(16);

    intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 5, 6);

    limelightSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 2, 2);

    bottomLimitSwitch = new DigitalInput(1);
    topLimitSwitch = new DigitalInput(0);
  }
  public void intakePeriodic(){
    intake();
    shootIndexManager();
  }

  private void intake() {
    if (Robot.xbox.getRightTriggerAxis() > 0 || intakeNow) {  // Right trigger held --> intake goes down and spins intake motor
      intakeSolenoid.set(Value.kReverse);
      intakeDrive.set(ControlMode.PercentOutput, calculateIntakeSpeed(Robot.SWERVEDRIVE.getCurrentSpeed()));
      turnToIntake();

    } else { 
      intakeSolenoid.set(Value.kForward); // Pulls intake back up and stops spinning
      intakeDrive.set(ControlMode.PercentOutput, 0);
    }
  }

  // Made this linear for now, can make more complex equation in the future
  private double calculateIntakeSpeed(double robotSpeed) {
    // y = 0.2x + 0.3
    double intakeSpeed = 0.2*(Math.abs(robotSpeed)) + 0.3;
    return intakeSpeed;
  }

  private void shootIndexManager() {
    if (Robot.SHOOTER.shooterAtSpeed() && (Robot.xbox.getLeftTriggerAxis() > 0 || shootNow)) {  // Once shooter gets up to speed AND left trigger held, balls fed to shooter
      if (!bottomLimitSwitch.get() && !topLimitSwitch.get()) {
        indexTop.set(ControlMode.PercentOutput, 0.2); // If there's only one ball being shot
      } else {
        indexTop.set(ControlMode.PercentOutput, 0.2); // If two balls are being shot
        indexBottom.set(ControlMode.PercentOutput, 0.2);
      }
    } else {
      indexerDrive(); // Defaults to state machine below
    }
  }

  private void indexerDrive() {
      switch (indexManager()) {
        case "1 Ball": // Hold the ball at the top of tower
          indexBottom.set(ControlMode.PercentOutput, 0.1);
          indexTop.set(ControlMode.PercentOutput, 0);
          break;

        case "2 Balls": // Indexer full
          indexBottom.set(ControlMode.PercentOutput, 0); 
          indexTop.set(ControlMode.PercentOutput, 0);
          break;

        case "Cargo in Transit":  // Bring ball from intake to top of tower
          indexTop.set(ControlMode.PercentOutput, 0.2);
          indexBottom.set(ControlMode.PercentOutput, 0.2);
          break;

        case "Reverse Intake":  // Both intakes go backwards
          indexTop.set(ControlMode.PercentOutput, -0.2);
          indexBottom.set(ControlMode.PercentOutput, -0.2);
          break;

        case "Intake Ball": // Spins bottom intake while ball in being intaked
          indexBottom.set(ControlMode.PercentOutput, 0.2);
          break;

        default:  // Everything stops
          indexBottom.set(ControlMode.PercentOutput, 0);
          indexTop.set(ControlMode.PercentOutput, 0);
          break;
      }
    } 

  private String indexManager() {
    if (Robot.xbox.getRightStickButtonPressed()) {
      return "Reverse Intake";
    } else if (!bottomLimitSwitch.get() && !topLimitSwitch.get()) { // Both switches pressed
      return "2 Balls";
    } else if (!bottomLimitSwitch.get() || (cargoInTransit && bottomLimitSwitch.get() && topLimitSwitch.get())) { // Bottom switch pressed/cargo in transit
      cargoInTransit = true;
      return "Cargo in Transit";
    } else if (!topLimitSwitch.get()) { //top switch pressed
      cargoInTransit = false;
      return "1 Ball";
    } else if (bottomLimitSwitch.get() && Robot.xbox.getRightTriggerAxis() > 0) { // Right trigger held AND nothing pressing the bottom switch
      return "Intake Ball";
    } else {
      return "default";
    }
  }

  public void turnToIntake() {
    limelightSolenoid.set(Value.kReverse);
    // Make intake pipeline
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);
    limelightIsOpen = false;
    // ^^^
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
