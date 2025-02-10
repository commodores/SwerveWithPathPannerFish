// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;


public class Elevator extends SubsystemBase {

  private TimeOfFlight elevatorSensor;

  private SparkFlex elevatorMotor =
      new SparkFlex(Constants.ArmivatorConstants.elevatorMotor, MotorType.kBrushless);
  //private SparkClosedLoopController elevatorClosedLoopController = elevatorMotor.getClosedLoopController();
  private RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();

  private boolean wasResetByTOF = false;

  /** Creates a new Elevator. */
  public Elevator() {
    elevatorSensor = new TimeOfFlight(4);
    elevatorSensor.setRangingMode(RangingMode.Short, 24);

    elevatorMotor.configure(
        Configs.ArmivatorSubsystem.elevatorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

        elevatorEncoder.setPosition(0);
  }

  public double getElevatorDistance(){

    return elevatorSensor.getRange();  
  
  }

  /** Zero the elevator encoder when the limit switch is pressed. */
  private void zeroElevatorOnLimitSwitch() {
    if (!wasResetByTOF && getElevatorDistance() < 58) {
      // Zero the encoder only when the limit switch is switches from "unpressed" to "pressed" to
      // prevent constant zeroing while pressed
      elevatorEncoder.setPosition(0);
      wasResetByTOF = true;
    } else if (getElevatorDistance()>=58) {
      wasResetByTOF = false;
    }
  }

 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    zeroElevatorOnLimitSwitch();
    SmartDashboard.putNumber("Elevator Sensor", getElevatorDistance());
    SmartDashboard.putNumber("Coral/Elevator/Actual Position", elevatorEncoder.getPosition());


  }
}
