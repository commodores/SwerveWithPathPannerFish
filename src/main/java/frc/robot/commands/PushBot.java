// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PushBot extends Command {
  private Timer backTimer;
  private CommandSwerveDrivetrain drivebase;
  private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();
  /** Creates a new pushBot. */
  public PushBot(CommandSwerveDrivetrain drivebase) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivebase = drivebase;
    addRequirements(drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.backTimer = new Timer();
    this.backTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
          drivebase.setControl(drive
        .withVelocityX(0.5) // Move towards reef
        .withVelocityY(0) // Strafe to branch
        .withRotationalRate(0));// Rotation towards target
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return backTimer.hasElapsed(.5);
  }
}
