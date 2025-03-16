// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Level1ArmSetpoints;
import frc.robot.subsystems.Level1Arm;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Level1ScorePosition extends Command {
  public Level1Arm m_Level1;
  /** Creates a new Level1Score. */
  public Level1ScorePosition(Level1Arm lvlOneSub) {

    m_Level1 = lvlOneSub;
    addRequirements(m_Level1);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
        m_Level1.resetAndMoveArmToAngle(Level1ArmSetpoints.kScore);
     m_Level1.setArmSetpoint(2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
