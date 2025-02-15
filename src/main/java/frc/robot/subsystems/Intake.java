// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final  SparkFlex intakeMotor = new SparkFlex(Constants.IntakeConstants.intakeMotor, MotorType.kBrushless);
  private final  SparkFlex hopperMotor = new SparkFlex(Constants.IntakeConstants.hopperMotor, MotorType.kBrushless);
      
  private LaserCan outSensor; 
  private LaserCan inSensor;

  //public LaserCan.Measurement measurementOutSensor; 
  public LaserCan.Measurement measurementInSensor;
  public LaserCan.Measurement measurementOutSensor;

  public Intake() {

 intakeMotor.configure(
        Configs.ArmivatorSubsystem.intakeConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

  hopperMotor.configure(
          Configs.ArmivatorSubsystem.hopperConfig,
          ResetMode.kResetSafeParameters,
          PersistMode.kPersistParameters);

  outSensor = new LaserCan (8);
  inSensor = new LaserCan (7);


  inSensor.getMeasurement();
  outSensor.getMeasurement();
  
  try {
    outSensor.setRangingMode(LaserCan.RangingMode.SHORT);
    outSensor.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
    outSensor.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    inSensor.setRangingMode(LaserCan.RangingMode.SHORT);
    inSensor.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
    inSensor.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
  } catch (ConfigurationFailedException e) {
    System.out.println("Configuration Failed!" +e);
  }

  
  
  }



    public void runIntakeSpeed(double speed){
      intakeMotor.set(speed);
    }

    public void runHopperSpeed(double speed){
      hopperMotor.set(speed);
    }

   public double getOutSensorDistance(){
      LaserCan.Measurement measurementOutSensor = outSensor.getMeasurement();
      return measurementOutSensor.distance_mm;
    }

    public double getInSensorDistance(){
      LaserCan.Measurement measurementInSensor = inSensor.getMeasurement();
      return measurementInSensor.distance_mm;
    }

    


  @Override
  public void periodic() {

    // This method will be called once per scheduler run
   // LaserCan.Measurement measurementOutSensor = outSensor.getMeasurement();
    SmartDashboard.putNumber("Out Sensor", getOutSensorDistance());
    SmartDashboard.putNumber("In Sensor", getInSensorDistance());
 
   
    }
  }

