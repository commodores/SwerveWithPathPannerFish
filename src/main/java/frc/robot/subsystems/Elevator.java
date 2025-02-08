// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Elevator extends SubsystemBase {

  private TimeOfFlight elevatorSensor;
  /** Creates a new Elevator. */
  public Elevator() {
    elevatorSensor = new TimeOfFlight(4);
    elevatorSensor.setRangingMode(RangingMode.Short, 24);
  }

  public double getElevatorDistance(){

  return elevatorSensor.getRange();
  
  
  }

 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Elevator Sensor", getElevatorDistance());
    


  }
}
