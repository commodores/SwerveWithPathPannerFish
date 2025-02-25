// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmSetpoints;
import frc.robot.Constants.ElevatorSetpoints;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Armivator;
import frc.robot.subsystems.Armivator.Setpoint;
import frc.robot.subsystems.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LevelOnePose extends Command {
  private final Armivator m_Armivator;
  /** Creates a new LevelOnePose. */
  public LevelOnePose(Armivator armivatorSub) {
    m_Armivator = armivatorSub;
    addRequirements(m_Armivator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_Armivator.setSetpointCommandNew(Armivator.Setpoint.kLevel1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    m_Armivator.setSetpointCommandNew(Armivator.Setpoint.kLevel1);
  
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
