// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CANdleSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoLEDTarget extends Command {

  private final CANdleSubsystem m_CANdle;
  private static final double negativeOffset = -6.5;
  private static final double positiveOffset = 6.5;
  double error;

  Optional<Alliance> ally;
  /** Creates a new AutoLEDTarget. */
  public AutoLEDTarget(CANdleSubsystem CANdle) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_CANdle = CANdle;
    addRequirements(m_CANdle);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if(ally.get() == Alliance.Red) {
        m_CANdle.setColor(255,0,0);
      }
      if(ally.get() == Alliance.Blue) {
        m_CANdle.setColor(0,0,255);
      }

    }
    else {
      m_CANdle.setColor(200,200,0);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    error = RobotContainer.m_Limelight.getX();
    //Check error
    if(RobotContainer.m_Limelight.seesTarget()){
      if(error > -3 && error < 3){
        m_CANdle.setColor(0, 255, 0);
      } else {
        if (ally.get() == Alliance.Red) {
          m_CANdle.setColor(255, 0, 0);
        }
        if (ally.get() == Alliance.Blue) {
          m_CANdle.setColor(0, 0, 255);
        }
      }
    }
   /*  if(RobotContainer.m_Limelight.seesTarget()){
      if ((error > negativeOffset - 3 && error < negativeOffset + 3) ||
      (error < positiveOffset - 3 && error > positiveOffset + 3)) {
        m_CANdle.setColor(0, 255, 0);
      } else {
        if(ally.isPresent()) {
          if (ally.get() == Alliance.Red) {
            m_CANdle.setColor(255, 0, 0); 
          } else if (ally.get() == Alliance.Blue) {
            m_CANdle.setColor(0, 0, 255);
          }
        }
      }
    }*/
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
