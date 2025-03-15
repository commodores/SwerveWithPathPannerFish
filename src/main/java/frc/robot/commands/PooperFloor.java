// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmSetpoints;
import frc.robot.Constants.ElevatorSetpoints;
import frc.robot.Constants.PooperArmSetpoints;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pooper;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PooperFloor extends Command {
   private final Pooper m_Pooper;
  /** Creates a new PooperFloor. */
  public PooperFloor(Pooper lvlOneSub) {

    m_Pooper = lvlOneSub;
    addRequirements(m_Pooper);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

     m_Pooper.resetAndMoveArmToAngle(PooperArmSetpoints.kFloor);
     m_Pooper.setArmSetpoint(1);
  
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
