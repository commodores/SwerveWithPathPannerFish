// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.ArmSetpoints;
import frc.robot.Constants.ElevatorSetpoints;

public class Armivator extends SubsystemBase {

  public enum Setpoint {
    kFeederStation,
    kNeutralPosition,
    kLevel1,
    kLevel2,
    kLevel3,
    kLevel4;
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
  private boolean wasResetByTOF = false;
  private double armCurrentTarget = ArmSetpoints.kFeederStation;
  private double elevatorCurrentTarget = ElevatorSetpoints.kFeederStation;
  private TimeOfFlight elevatorSensor;

 
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

    return (elevatorSensor.getRange()*0.03937008)-1;
        
  }

  private double calculateFeedforward(double armAngleRadians) {
    double kG = 0.05; // Gravity compensation gain
    return kG * Math.cos(armAngleRadians - Math.PI / 4);
  }

  private void moveToSetpoint() {
    double feedforward = calculateFeedforward(armCurrentTarget);
    armController.setReference(armCurrentTarget, ControlType.kMAXMotionPositionControl,ClosedLoopSlot.kSlot0,feedforward);
    elevatorClosedLoopController.setReference(
        elevatorCurrentTarget, ControlType.kMAXMotionPositionControl);
  }

  /** Zero the elevator encoder when the limit switch is pressed. */
  private void zeroElevatorOnLimitSwitch() {
    if (!wasResetByTOF && getElevatorDistanceInInch() < 1) {
      // Zero the encoder only when the limit switch is switches from "unpressed" to "pressed" to
      // prevent constant zeroing while pressed
      elevatorEncoder.setPosition(0);
      wasResetByTOF = true;
    } else if (getElevatorDistanceInInch()>=1) {
      wasResetByTOF = false;
    }
  }

    /**
   * Command to set the subsystem setpoint. This will set the arm and elevator to their predefined
   * positions for the given setpoint.
   */
  public Command setSetpointCommandNew(Setpoint setpoint) {
    return this.runOnce(
        () -> {
          //Feeder Station
          if(armCurrentTarget == ArmSetpoints.kNeutralPosition && setpoint == Setpoint.kFeederStation){
            armCurrentTarget = ArmSetpoints.kFeederStation;
            elevatorCurrentTarget = ElevatorSetpoints.kFeederStation;
          }
          //Neutral Position
          if(setpoint == Setpoint.kNeutralPosition){
            armCurrentTarget = ArmSetpoints.kNeutralPosition;
            elevatorCurrentTarget = ElevatorSetpoints.kNeutralPosition;
          }
          //L1
          if(setpoint == Setpoint.kLevel1){
            armCurrentTarget = ArmSetpoints.kLevel1;
            elevatorCurrentTarget = ElevatorSetpoints.kLevel1;
          }
          //L2
          if(setpoint == Setpoint.kLevel2){
            armCurrentTarget = ArmSetpoints.kLevel2;
            elevatorCurrentTarget = ElevatorSetpoints.kLevel2;
          }

          //L3
          if(setpoint == Setpoint.kLevel3){
            armCurrentTarget = ArmSetpoints.kLevel3;
            elevatorCurrentTarget = ElevatorSetpoints.kLevel3;
          }

          //L4
          if(setpoint == Setpoint.kLevel4){
            armCurrentTarget = ArmSetpoints.kLevel4;
            elevatorCurrentTarget = ElevatorSetpoints.kLevel4;
          }
        });
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    moveToSetpoint();
    zeroElevatorOnLimitSwitch();

    // Display subsystem values
    SmartDashboard.putNumber("Coral/Arm/Target Position", armCurrentTarget);
    SmartDashboard.putNumber("Coral/Arm/Actual Position", armEncoder.getPosition());
    SmartDashboard.putNumber("Elevator Sensor", getElevatorDistanceInInch());
    SmartDashboard.putNumber("Coral/Elevator/Target Position", elevatorCurrentTarget);
    SmartDashboard.putNumber("Coral/Elevator/Actual Position", elevatorEncoder.getPosition());
  }
}
