// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlignToBranchConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AlignToReefTagRelative extends Command {
  private PIDController xController, yController, rotController;
  private boolean isRightScore;
  private Timer dontSeeTagTimer, stopTimer, totalTimer;
  private CommandSwerveDrivetrain drivebase;
  private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();
  private double tagID = -1;
  private double rampTime = 0.5; // seconds to full speed

  public AlignToReefTagRelative(boolean isRightScore, CommandSwerveDrivetrain drivebase) {
    xController = new PIDController(AlignToBranchConstants.X_REEF_ALIGNMENT_P, 0.0, 0);     // Move towards reef
    yController = new PIDController(AlignToBranchConstants.Y_REEF_ALIGNMENT_P, 0.0, 0);     // Strafe to branch
    rotController = new PIDController(AlignToBranchConstants.ROT_REEF_ALIGNMENT_P, 0, 0);   // Rotation towards target
    this.isRightScore = isRightScore;
    this.drivebase = drivebase;
    addRequirements(drivebase);
  }

  @Override
  public void initialize() {
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();
    this.totalTimer = new Timer();
    this.totalTimer.start();

    // Rotation towards target
    rotController.setSetpoint(AlignToBranchConstants.ROT_SETPOINT_REEF_ALIGNMENT);
    rotController.setTolerance(AlignToBranchConstants.ROT_TOLERANCE_REEF_ALIGNMENT);
    // Move towards reef
    xController.setSetpoint(AlignToBranchConstants.X_SETPOINT_REEF_ALIGNMENT); 
    xController.setTolerance(AlignToBranchConstants.X_TOLERANCE_REEF_ALIGNMENT);
    // Strafe to branch
    yController.setSetpoint(isRightScore ? AlignToBranchConstants.Y_SETPOINT_REEF_ALIGNMENT_RIGHT : AlignToBranchConstants.Y_SETPOINT_REEF_ALIGNMENT_LEFT); 
    yController.setTolerance(AlignToBranchConstants.Y_TOLERANCE_REEF_ALIGNMENT);

    tagID = LimelightHelpers.getFiducialID("limelight");
  }

  @Override
  public void execute() {
    if (LimelightHelpers.getTV("limelight") && LimelightHelpers.getFiducialID("limelight") == tagID) {
      this.dontSeeTagTimer.reset();

      double[] postions = LimelightHelpers.getBotPose_TargetSpace("limelight");
      SmartDashboard.putNumber("tx", postions[0]);
      SmartDashboard.putNumber("tz", postions[2]);
      SmartDashboard.putNumber("ry", postions[4]);

      double ramp = Math.min(1.0, totalTimer.get() / rampTime);
      double xSpeed = xController.calculate(postions[2])*ramp;       // Move towards reef
      double ySpeed = -yController.calculate(postions[0]);      // Strafe to branch
      double rotValue = -rotController.calculate(postions[4]);  // Rotation towards target

      drivebase.setControl(drive
        .withVelocityX(xSpeed) // Move towards reef
        .withVelocityY(ySpeed) // Strafe to branch
        .withRotationalRate(rotValue));// Rotation towards target

    if (!rotController.atSetpoint() ||
        !yController.atSetpoint() ||
        !xController.atSetpoint()) {
      stopTimer.reset();
    }
     
     else {
      drivebase.setControl(drive
        .withVelocityX(0)
        .withVelocityY(0)
        .withRotationalRate(0));
    }

      SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
    }
  }

  @Override
  public void end(boolean interrupted) {
    drivebase.setControl(drive
        .withVelocityX(0) // Drive forward with negative Y(forward)
        .withVelocityY(0) // Drive left with negative X (left)
        .withRotationalRate(0));
  }

  @Override
  public boolean isFinished() {
    // Requires the robot to stay in the correct position for 0.3 seconds, as long as it gets a tag in the camera
    return this.dontSeeTagTimer.hasElapsed(AlignToBranchConstants.DONT_SEE_TAG_WAIT_TIME) ||
        stopTimer.hasElapsed(AlignToBranchConstants.POSE_VALIDATION_TIME) ||
        totalTimer.hasElapsed(AlignToBranchConstants.POSE_TOTAL_TIME);
  }
}