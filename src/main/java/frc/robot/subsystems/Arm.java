// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.ArmSetpoints;
import frc.robot.Constants.ElevatorSetpoints;
import frc.robot.subsystems.Armivator.Setpoint;


public class Arm extends SubsystemBase {

  // Initialize arm SPARK. We will use MAXMotion position control for the arm, so we also need to
  // initialize the closed loop controller and encoder.
 // private SparkFlex armMotor =
      //new SparkFlex(Constants.ArmivatorConstants.armMotor, MotorType.kBrushless);
  //private SparkClosedLoopController armController = armMotor.getClosedLoopController();
 // private RelativeEncoder armEncoder = armMotor.getEncoder();

 public enum Setpoint {
  kFeederStation,
  kLevel1,
  kLevel2,
  kLevel3,
  kLevel4;
 }
   private double armCurrentTarget = ArmSetpoints.kFeederStation;
  /*
  * Creates a new Arm. */

  private SparkFlex armMotor =
      new SparkFlex(Constants.ArmivatorConstants.armMotor, MotorType.kBrushless);
  private SparkClosedLoopController armController = armMotor.getClosedLoopController();
  private RelativeEncoder armEncoder = armMotor.getEncoder();

  public Arm() {

    armMotor.configure(
        Configs.ArmivatorSubsystem.armConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Zero arm and elevator encoders on initialization
    armEncoder.setPosition(0);
  }

  private void moveToSetpoint() {
    armController.setReference(armCurrentTarget, ControlType.kMAXMotionPositionControl);
  }

public Command setSetpointCommand(Setpoint setpoint) {
    return this.runOnce(
        () -> {
          switch (setpoint) {
            case kFeederStation:
              armCurrentTarget = ArmSetpoints.kFeederStation;
              break;
            case kLevel1:
              armCurrentTarget = ArmSetpoints.kLevel1;
              break;
            case kLevel2:
              armCurrentTarget = ArmSetpoints.kLevel2;
              break;
            case kLevel3:
              armCurrentTarget = ArmSetpoints.kLevel3;
              break;
            case kLevel4:
              armCurrentTarget = ArmSetpoints.kLevel4;
              break;
          }
    
        });
  
      }


    
    
 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Coral/Arm/Actual Position", armEncoder.getPosition());
  }
}
