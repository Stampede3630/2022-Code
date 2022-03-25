package frc.robot;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.networktables.NetworkTableInstance;

import com.revrobotics.ColorSensorV3;

public class Intake implements Loggable {
  
  private static Intake SINGLE_INSTANCE = new Intake();
  
  public WPI_TalonFX intakeDrive;
  public WPI_TalonFX indexBottom;
  public WPI_TalonFX indexTop;
  public DoubleSolenoid intakeSolenoid;
  public DoubleSolenoid limelightSolenoid;
  // SWITCHES: GREEN = NOT PRESSED, RED = PRESSED
  // SWITCHES RETURN TRUE WHEN NOT PRESSED, FALSE WHEN PRESSED
  public DigitalInput bottomLimitSwitch;
  public DigitalInput topLimitSwitch;
  public ColorSensorV3 colorSensor;
  private boolean cargoInTransit = false;
  @Log(tabName = "CompetitionLogger", rowIndex = 2, columnIndex = 3)
  public boolean intakeNow = false;
  @Log(tabName = "CompetitionLogger", rowIndex = 1, columnIndex = 3)
  public boolean shootNow = false;
  public boolean limelightIsOpen = true; // rename and figure out if it starts open or closed
  public boolean intakeIsOut = false;
  public double intakeSpeed = -18000;
  public boolean ballReject = true;
  public String indexState="";
  
  public final I2C.Port i2cPort = I2C.Port.kMXP;

  public static Intake getInstance() {
    return SINGLE_INSTANCE;
  }

  public void init(){

    //SJV: PUT ALL SOLENOID AND MOTOR VALUES IN CONSTANTS, FORMATE BELOW SHOULD LOOK LIKE "=new WPI_TalonFX(Constants.intakeMotor);"
    indexBottom = new WPI_TalonFX(6);
    indexTop = new WPI_TalonFX(9);

    indexBottom.setNeutralMode(NeutralMode.Brake);
    indexTop.setNeutralMode(NeutralMode.Brake);

    intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 4, 5);

    limelightSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);

    bottomLimitSwitch = new DigitalInput(0);
    topLimitSwitch = new DigitalInput(1);

    colorSensor = new ColorSensorV3(i2cPort);

    intakeDrive = new WPI_TalonFX(7);
        intakeDrive.setInverted(TalonFXInvertType.Clockwise);
        intakeDrive.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 20);
        intakeDrive.config_kF(0,
                1023 * 0.75 / 20000, 20);

        intakeDrive.config_kP(0,
                0.055, 20);


    
  }
  public void intakePeriodic(){
    intake();
    shootIndexManager();
    indexState = indexManager();
  }
//SJV: WE MAY NEED TO RUN INTAKE ON A PID SO IT GETS TO SPEED A LOT FASTER 
  private void intake() {
    if (Robot.xbox.getRightTriggerAxis() > 0 || intakeNow || indexManager() == "Ball Reject") {  // Right trigger held --> intake goes down and spins intake motor
      if (!intakeIsOut) {
        intakeSolenoid.set(Value.kForward);
        intakeIsOut = true;
      }
      
      intakeDrive.set(ControlMode.Velocity, intakeSpeed); //-18000

      turnToIntake();

    } else {
      if (intakeIsOut) {
        intakeSolenoid.set(Value.kReverse);
        intakeIsOut = false; // Pulls intake back up and stops spinning
      } 

      intakeDrive.set(ControlMode.PercentOutput, 0);
    }
  }

  private void shootIndexManager() {
    if (Robot.xbox.getLeftBumper() || (Robot.SHOOTER.shooterAtSpeed() && (Robot.xbox.getLeftTriggerAxis() > 0 || shootNow))) {  // Once shooter gets up to speed AND left trigger held, balls fed to shooter
      if (!bottomLimitSwitch.get() && !topLimitSwitch.get()) {
        indexTop.set(ControlMode.PercentOutput, -0.2); // If there's only one ball being shot
      } else {
        indexTop.set(ControlMode.PercentOutput, -0.2); // If two balls are being shot
        indexBottom.set(ControlMode.PercentOutput, -0.2);
      }
    } else {
      indexerDrive(); // Defaults to state machine below
    }
  }
  //SJV: DOES INDEXER NEED TO BE RUN FASTER?
  private void indexerDrive() {
      switch (indexState) {
        case "1 Ball": // Hold the ball at the top of tower
          indexBottom.set(ControlMode.PercentOutput, -0.34);
          indexTop.set(ControlMode.PercentOutput, 0);
          break;

        case "2 Balls": // Indexer full
          indexBottom.set(ControlMode.PercentOutput, 0); 
          indexTop.set(ControlMode.PercentOutput, 0);
          break;

        case "Cargo in Transit":  // Bring ball from intake to top of tower
          indexTop.set(ControlMode.PercentOutput, -0.4);
          indexBottom.set(ControlMode.PercentOutput, -0.4);
          break;

        case "Reverse Intake":  // Both intakes go backwards
          indexTop.set(ControlMode.PercentOutput, 0.8);
          indexBottom.set(ControlMode.PercentOutput, 0.8);
          break;

        case "Intake Ball": // Spins bottom intake while ball in being intaked
          indexBottom.set(ControlMode.PercentOutput, -0.4);
          break;

        case "Ball Reject": //Rejects ball if it's the wrong color
        indexBottom.set(ControlMode.PercentOutput, 0.8);
          break;

        default:  // Everything stops
          indexBottom.set(ControlMode.PercentOutput, 0);
          indexTop.set(ControlMode.PercentOutput, 0);
          break;
      }
    } 

  private String indexManager() {
    if (Robot.xbox.getBButton()) {
      return "Reverse Intake";
    } else if(DriverStation.getAlliance() == Alliance.Blue && (colorSensor.getBlue() < colorSensor.getRed()) && colorSensor.getRed() > 1000 && ballReject) {
      return "Ball Reject";
    } else if(DriverStation.getAlliance() == Alliance.Red && (colorSensor.getRed() < colorSensor.getBlue()) && colorSensor.getBlue() > 1000 && ballReject) {
      return "Ball Reject";
    } else if (!bottomLimitSwitch.get() && !topLimitSwitch.get()) { // Both switches pressed
      return "2 Balls";
    } else if (!bottomLimitSwitch.get() || (cargoInTransit && bottomLimitSwitch.get() && topLimitSwitch.get())) { // Bottom switch pressed/cargo in transit
      cargoInTransit = true;
      return "Cargo in Transit";
    } else if (!topLimitSwitch.get()) { //top switch pressed
      cargoInTransit = false;
      return "1 Ball";
    } else if (bottomLimitSwitch.get() && (Robot.xbox.getRightTriggerAxis() > 0 || intakeNow )) { // Right trigger held AND nothing pressing the bottom switch
      return "Intake Ball";
    } else {
      return "default";
    }
  }

  public void turnToIntake() {
    if (limelightIsOpen) {
      //limelightSolenoid.set(Value.kReverse);
    }
    // Make intake pipeline
    //SJV: time to make those pipe lines
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
    limelightIsOpen = false;
    // ^^^
  }
  
  public void checkAndSetIntakeCANStatus() {
    if(indexBottom.hasResetOccurred()){
      int mycounter=0;
      if(indexBottom.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255,1000) !=ErrorCode.OK) {mycounter++;}
      if(indexBottom.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255,1000)!=ErrorCode.OK) {mycounter++;}
      if(indexBottom.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255,1000)!=ErrorCode.OK) {mycounter++;}
      if(indexBottom.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255,1000)!=ErrorCode.OK) {mycounter++;}
      if(indexBottom.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 255,1000)!=ErrorCode.OK) {mycounter++;}
      if(indexBottom.setStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus, 255,1000)!=ErrorCode.OK) {mycounter++;}
      if(indexBottom.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 255,1000)!=ErrorCode.OK) {mycounter++;}
      if(indexBottom.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 255,1000)!=ErrorCode.OK) {mycounter++;}
      if(indexBottom.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 255,1000)!=ErrorCode.OK) {mycounter++;}
      if(indexBottom.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 255,1000)!=ErrorCode.OK) {mycounter++;}
      if(indexBottom.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 255,1000)!=ErrorCode.OK) {mycounter++;}
      if(indexBottom.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 255,1000)!=ErrorCode.OK) {mycounter++;}
      if(indexBottom.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 255,1000)!=ErrorCode.OK) {mycounter++;}
      if(indexBottom.setStatusFramePeriod(StatusFrameEnhanced.Status_15_FirmwareApiStatus, 255,1000)!=ErrorCode.OK) {mycounter++;}
      System.out.println("RESET DETECTED FOR TALONFX " + indexBottom.getDeviceID() + " Errors: " + mycounter);
    }
    
    if(intakeDrive.hasResetOccurred()){
      int mycounter=0;
      if(intakeDrive.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255,1000) !=ErrorCode.OK) {mycounter++;}
      if(intakeDrive.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20,100)!=ErrorCode.OK) {mycounter++;}
      if(intakeDrive.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255,100)!=ErrorCode.OK) {mycounter++;}
      if(intakeDrive.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255,100)!=ErrorCode.OK) {mycounter++;}
      if(intakeDrive.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 255,100)!=ErrorCode.OK) {mycounter++;}
      if(intakeDrive.setStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus, 255,100)!=ErrorCode.OK) {mycounter++;}
      if(intakeDrive.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 255,100)!=ErrorCode.OK) {mycounter++;}
      if(intakeDrive.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 255,100)!=ErrorCode.OK) {mycounter++;}
      if(intakeDrive.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 255,100)!=ErrorCode.OK) {mycounter++;}
      if(intakeDrive.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 255,100)!=ErrorCode.OK) {mycounter++;}
      if(intakeDrive.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 255,100)!=ErrorCode.OK) {mycounter++;}
      if(intakeDrive.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 255,100)!=ErrorCode.OK) {mycounter++;}
      if(intakeDrive.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 255,100)!=ErrorCode.OK) {mycounter++;}
      if(intakeDrive.setStatusFramePeriod(StatusFrameEnhanced.Status_15_FirmwareApiStatus, 255,100)!=ErrorCode.OK) {mycounter++;}
      System.out.println("RESET DETECTED FOR TALONFX " + intakeDrive.getDeviceID() + " Errors: " + mycounter);
    }
    
    if(indexTop.hasResetOccurred()){
      int mycounter=0;
      if(indexTop.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255,1000) !=ErrorCode.OK) {mycounter++;}
      if(indexTop.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255,100)!=ErrorCode.OK) {mycounter++;}
      if(indexTop.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255,100)!=ErrorCode.OK) {mycounter++;}
      if(indexTop.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255,100)!=ErrorCode.OK) {mycounter++;}
      if(indexTop.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 255,100)!=ErrorCode.OK) {mycounter++;}
      if(indexTop.setStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus, 255,100)!=ErrorCode.OK) {mycounter++;}
      if(indexTop.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 255,100)!=ErrorCode.OK) {mycounter++;}
      if(indexTop.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 255,100)!=ErrorCode.OK) {mycounter++;}
      if(indexTop.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 255,100)!=ErrorCode.OK) {mycounter++;}
      if(indexTop.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 255,100)!=ErrorCode.OK) {mycounter++;}
      if(indexTop.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 255,100)!=ErrorCode.OK) {mycounter++;}
      if(indexTop.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 255,100)!=ErrorCode.OK) {mycounter++;}
      if(indexTop.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 255,100)!=ErrorCode.OK) {mycounter++;}
      if(indexTop.setStatusFramePeriod(StatusFrameEnhanced.Status_15_FirmwareApiStatus, 255,100)!=ErrorCode.OK) {mycounter++;}
      System.out.println("RESET DETECTED FOR TALONFX " + indexTop.getDeviceID() + " Errors:" + mycounter);
    }
  }

  //I think that matters... not SURE THOWRONG WRONG WATCH OBLOG WANTS DOUBLES i THINK

  
  // @Log
  // public double getRedColor() {
  //   return (double) colorSensor.getRed();
    
  // }

  // @Log
  // public double getBlueColor() {
  //   return (double) colorSensor.getBlue();
    
  // }

  // @Log
  // public double getGreenColor() {
  //   return (double) colorSensor.getGreen();
    
  // }
  
  // @Log.BooleanBox(rowIndex = 1, columnIndex = 2)
  // public boolean getBottomLimitSwitch() {
  //   return bottomLimitSwitch.get();
  // }
  
  // @Log.BooleanBox(rowIndex = 3, columnIndex = 4)
  // public boolean getTopLimitSwitch() {
  //   return topLimitSwitch.get();
  // }

  @Config(tabName = "CompetitionLogger", defaultValueBoolean = true, rowIndex = 3, columnIndex = 3, height = 1, width = 2)
  public void setBallReject(boolean _input) {
    ballReject = _input;
  }

  // @Config.NumberSlider(name = "Set Intake Speed", defaultValue = -15000, min = -20000, max = -2000, blockIncrement = 1000, rowIndex = 1, columnIndex = 0, height = 1, width = 3)
  // public void setIntakeSpeed(double targetIntakeVelocity) {
  //     intakeSpeed = targetIntakeVelocity;

  // }
  
  
}
