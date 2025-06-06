// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoReverse extends Command {

  private final Intake m_Intake;

  /** Creates a new AutoIntake. */
  public AutoReverse(Intake intakeSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Intake = intakeSub;
    addRequirements(m_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Intake.runIntakeSpeed(-0.1);//.08
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {  
    m_Intake.runIntakeSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_Intake.getInSensorDistance() < 100 && m_Intake.getOutSensorDistance() < 100;
  }
}
