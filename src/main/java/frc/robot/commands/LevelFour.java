// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmSetpoints;
import frc.robot.Constants.ElevatorSetpoints;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LevelFour extends Command {
  
  private final Elevator m_Elevator;
  private final Arm m_Arm;

  /** Creates a new LevelOnePose. */
  public LevelFour(Arm armSub, Elevator elevatorSub) {
    m_Arm = armSub;
    m_Elevator = elevatorSub;
    addRequirements(m_Arm, m_Elevator);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
     //Level 4
     m_Arm.resetAndMoveArmToAngle(ArmSetpoints.kLevel4);
     m_Elevator.setNewHeight(ElevatorSetpoints.kLevel4);
     m_Arm.setArmSetpoint(4);
     m_Elevator.setElevatorSetpoint(4);  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
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
