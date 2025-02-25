// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volt;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.ArmSetpoints;
import frc.robot.Constants.ElevatorSetpoints;

public class Armivator extends SubsystemBase {

  public enum Setpoint {
    kFeederStation,
    kLevel1,
    kLevel2,
    kLevel3,
    kLevel4,
    kAlgaeLow,
    kAlgaeHigh;
  }

  // Initialize arm SPARK. We will use MAXMotion position control for the arm, so we also need to
  // initialize the closed loop controller and encoder.
  private SparkFlex armMotor =
      new SparkFlex(Constants.ArmivatorConstants.armMotor, MotorType.kBrushless);
  private SparkClosedLoopController armController = armMotor.getClosedLoopController();
  private AbsoluteEncoder armEncoder = armMotor.getAbsoluteEncoder();

  // Initialize elevator SPARK. We will use MAXMotion position control for the elevator, so we also
  // need to initialize the closed loop controller and encoder.
  private SparkFlex elevatorMotor =
      new SparkFlex(Constants.ArmivatorConstants.elevatorMotor, MotorType.kBrushless);
  private SparkClosedLoopController elevatorClosedLoopController =
      elevatorMotor.getClosedLoopController();
  private RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();

  // Member variables for subsystem state management
  private boolean wasZeroResetByTOF = false;
  private double armCurrentTarget = ArmSetpoints.kFeederStation;
  private double elevatorCurrentTarget = ElevatorSetpoints.kFeederStation;
  private TimeOfFlight elevatorSensor;

  // ARM SysID Routine
  private final SysIdRoutine armSysIdRoutine = new SysIdRoutine(
    new SysIdRoutine.Config(
      null,
      Volt.of(4),
      null,
      null
    ),
    new SysIdRoutine.Mechanism(
        armMotor::setVoltage,
        null,
        this
    )
  );

  // Elevator SysID Routine
  private final SysIdRoutine elevatorSysIdRoutine = new SysIdRoutine(
    new SysIdRoutine.Config(
      null,
      Volt.of(4),
      null,
      null
    ),
    new SysIdRoutine.Mechanism(
        armMotor::setVoltage,
        null,
        this
    )
  );

 
  /** Creates a new Arm. */
  public Armivator() {

     /*
     * Apply the appropriate configurations to the SPARKs.
     *
     * kResetSafeParameters is used to get the SPARK to a known state. This
     * is useful in case the SPARK is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK loses power. This is useful for power cycles that may occur
     * mid-operation.
     */
     
    armMotor.configure(
        Configs.ArmivatorSubsystem.armConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    
    elevatorMotor.configure(
        Configs.ArmivatorSubsystem.elevatorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    elevatorSensor = new TimeOfFlight(4);
    elevatorSensor.setRangingMode(RangingMode.Short, 24);

    // Zero elevator encoder on initialization
    elevatorEncoder.setPosition(0);
  }

  public double getElevatorDistanceInInch(){

    return (elevatorSensor.getRange()*0.03937008)-2;
        
  }

  public double getElevatorDistanceInMeter(){

    return Units.inchesToMeters((elevatorSensor.getRange()*0.03937008)-2);
        
  }

  private double calculateArmFeedForward(){
    if(armCurrentTarget == ArmSetpoints.kFeederStation){
      return .05;
    } else if(armCurrentTarget == ArmSetpoints.kLevel1){
      return .35;
    } else if(armCurrentTarget == ArmSetpoints.kLevel2){
      return .2;
    } else if(armCurrentTarget == ArmSetpoints.kLevel3){
      return .2;
    } else if(armCurrentTarget == ArmSetpoints.kLevel4){
      return .05;
    } else if(armCurrentTarget == ArmSetpoints.kAlgaeLow){
      return .3;
    } else if(armCurrentTarget == ArmSetpoints.kAlgaeHigh){
      return .3;
    } else {
      return .1;
    }
      
  }

  private double calculateElevatorFeedForward(){
    return .5;
  }    

  private void moveToSetpoint() {
    armController.setReference(armCurrentTarget, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, calculateArmFeedForward());
    //elevatorClosedLoopController.setReference(elevatorCurrentTarget, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, calculateElevatorFeedForward());
    //armController.setReference(armCurrentTarget, ControlType.kMAXMotionPositionControl);
    elevatorClosedLoopController.setReference(elevatorCurrentTarget, ControlType.kMAXMotionPositionControl);
  }

  /** Zero the elevator encoder when the laser reads min. */
  private void zeroElevatorOnLaser() {
    if (!wasZeroResetByTOF && getElevatorDistanceInInch() < .2) {
      // Zero the encoder laser sees min to
      // prevent constant zeroing while retracted
      elevatorEncoder.setPosition(0);
      wasZeroResetByTOF = true;
    } else if (getElevatorDistanceInInch()>=2) {
      wasZeroResetByTOF = false;
    }      
  }

  public double getArmSetpoint() {
    return armCurrentTarget;
  }

  public double getElevatorSetpoint() {
    return elevatorCurrentTarget;
  }

   /**
   * Command to set the subsystem setpoint. This will set the arm and elevator to their predefined
   * positions for the given setpoint.
   */
  public Command setSetpointCommandNew(Setpoint setpoint) {
    return this.runOnce(
        () -> {
          //Feeder Station
          if(elevatorCurrentTarget == ElevatorSetpoints.kLevel1 && setpoint == Setpoint.kFeederStation){
            armCurrentTarget = ArmSetpoints.kFeederStation;
            elevatorCurrentTarget = ElevatorSetpoints.kFeederStation;
          }
    
          //L1
          if(setpoint == Setpoint.kLevel1){
            armCurrentTarget = ArmSetpoints.kLevel1;
            elevatorCurrentTarget = ElevatorSetpoints.kLevel1;
          }
          //L2
          if(armCurrentTarget != ArmSetpoints.kFeederStation && setpoint == Setpoint.kLevel2){
            armCurrentTarget = ArmSetpoints.kLevel2;
            elevatorCurrentTarget = ElevatorSetpoints.kLevel2;
          }

          //L3
          if(armCurrentTarget != ArmSetpoints.kFeederStation && setpoint == Setpoint.kLevel3){
            armCurrentTarget = ArmSetpoints.kLevel3;
            elevatorCurrentTarget = ElevatorSetpoints.kLevel3;
          }

          //L4
          if(armCurrentTarget != ArmSetpoints.kFeederStation && setpoint == Setpoint.kLevel4){
            armCurrentTarget = ArmSetpoints.kLevel4;
            elevatorCurrentTarget = ElevatorSetpoints.kLevel4;
          }

          //Algae Low
          if(armCurrentTarget != ArmSetpoints.kFeederStation && setpoint == Setpoint.kAlgaeLow){
            armCurrentTarget = ArmSetpoints.kAlgaeLow;
            elevatorCurrentTarget = ElevatorSetpoints.kAlgaeLow;
          }

          //Algae High
          if(armCurrentTarget != ArmSetpoints.kFeederStation && setpoint == Setpoint.kAlgaeHigh){
            armCurrentTarget = ArmSetpoints.kAlgaeHigh;
            elevatorCurrentTarget = ElevatorSetpoints.kAlgaeHigh;
          }
        });
  }

  public Command armSysIdQuasistatic(SysIdRoutine.Direction direction) {
      return armSysIdRoutine.quasistatic(direction);
  }

  public Command armSysIdDynamic(SysIdRoutine.Direction direction) {
      return armSysIdRoutine.dynamic(direction);
  }

  public Command elevatorSysIdQuasistatic(SysIdRoutine.Direction direction) {
      return elevatorSysIdRoutine.quasistatic(direction);
  }

  public Command elevatorSysIdDynamic(SysIdRoutine.Direction direction) {
      return elevatorSysIdRoutine.dynamic(direction);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    moveToSetpoint();
    zeroElevatorOnLaser();

    // Display subsystem values
    SmartDashboard.putNumber("Coral/Arm/Target Position", armCurrentTarget);
    SmartDashboard.putNumber("Coral/Arm/Actual Position", armEncoder.getPosition());
    SmartDashboard.putNumber("Elevator Sensor", getElevatorDistanceInInch());
    SmartDashboard.putNumber("Coral/Elevator/Target Position", elevatorCurrentTarget);
    SmartDashboard.putNumber("Coral/Elevator/Actual Position", elevatorEncoder.getPosition());
  }
}
