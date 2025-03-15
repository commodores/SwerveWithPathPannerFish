// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.PooperArmSetpoints;
import frc.robot.Constants.PooperConstants;

public class PooperIntake extends SubsystemBase {
    private final SparkFlex m_pooperIntakeMotor;
    
    public PooperIntake() {
      
        m_pooperIntakeMotor = new SparkFlex(PooperConstants.pooperIntakeMotor, MotorType.kBrushless);
        
        m_pooperIntakeMotor.configure(
        Configs.PooperSubsystem.pooperIntakeConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

      
        
    }

  
   

    @Override
    public void periodic() {

      
    }


  

    public void setIntakeSpeed(double speed) {
        m_pooperIntakeMotor.set(speed);
    }



}