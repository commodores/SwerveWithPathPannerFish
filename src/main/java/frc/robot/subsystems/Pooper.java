// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmivatorConstants;
import frc.robot.Constants.PooperConstants;

public class Pooper extends SubsystemBase {
 
  private final SparkFlex pooperArm = new SparkFlex(PooperConstants.pooperMotor, MotorType.kBrushless);
  private final SparkFlex pooperIntake = new SparkFlex(PooperConstants.pooperIntake, MotorType.kBrushless);
 
  

  /** Creates a new Pooper. */
  public Pooper() {
    
  }

  public void runPooper(double speed) {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
