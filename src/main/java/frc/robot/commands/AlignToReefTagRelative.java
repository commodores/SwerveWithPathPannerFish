// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.AlignToBranchConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AlignToReefTagRelative extends Command {
  private PIDController xController, yController, rotController;
  private boolean isRightScore;
  private Timer dontSeeTagTimer, stopTimer;
  private CommandSwerveDrivetrain drivetrain;
  private double tagID = -1;

  public AlignToReefTagRelative(boolean isRightScore, CommandSwerveDrivetrain drivetrain) {
    xController = new PIDController(AlignToBranchConstants.X_REEF_ALIGNMENT_P, 0.0, 0);  // Vertical movement
    yController = new PIDController(AlignToBranchConstants.Y_REEF_ALIGNMENT_P, 0.0, 0);  // Horitontal movement
    rotController = new PIDController(AlignToBranchConstants.ROT_REEF_ALIGNMENT_P, 0, 0);  // Rotation
    this.isRightScore = isRightScore;
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();

    rotController.setSetpoint(AlignToBranchConstants.ROT_SETPOINT_REEF_ALIGNMENT);
    rotController.setTolerance(AlignToBranchConstants.ROT_TOLERANCE_REEF_ALIGNMENT);

    xController.setSetpoint(AlignToBranchConstants.X_SETPOINT_REEF_ALIGNMENT);
    xController.setTolerance(AlignToBranchConstants.X_TOLERANCE_REEF_ALIGNMENT);

    yController.setSetpoint(isRightScore ? AlignToBranchConstants.Y_SETPOINT_REEF_ALIGNMENT : -AlignToBranchConstants.Y_SETPOINT_REEF_ALIGNMENT);
    yController.setTolerance(AlignToBranchConstants.Y_TOLERANCE_REEF_ALIGNMENT);

    tagID = LimelightHelpers.getFiducialID("");
  }

  @Override
  public void execute() {
    if (LimelightHelpers.getTV("") && LimelightHelpers.getFiducialID("") == tagID) {
      this.dontSeeTagTimer.reset();

      double[] postions = LimelightHelpers.getBotPose_TargetSpace("");
      SmartDashboard.putNumber("x", postions[2]);

      double xSpeed = xController.calculate(postions[2]);
      SmartDashboard.putNumber("xspee", xSpeed);
      double ySpeed = -yController.calculate(postions[0]);
      double rotValue = -rotController.calculate(postions[4]);

      // Apply movement correction
        drivetrain.applyRequest(() -> 
            new SwerveRequest.RobotCentric()
                .withVelocityX(xSpeed)
                .withVelocityY(ySpeed)
                .withRotationalRate(rotValue)
        ).schedule();

      //drivebase.drive(new Translation2d(xSpeed, ySpeed), rotValue, false);

      if (!rotController.atSetpoint() ||
          !yController.atSetpoint() ||
          !xController.atSetpoint()) {
        stopTimer.reset();
      }
    } else {
      drivetrain.applyRequest(() -> 
            new SwerveRequest.RobotCentric()
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0)
        ).schedule();
    }

    SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.applyRequest(() -> 
            new SwerveRequest.RobotCentric()
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0)
        ).schedule();
  }

  @Override
  public boolean isFinished() {
    // Requires the robot to stay in the correct position for 0.3 seconds, as long as it gets a tag in the camera
    return this.dontSeeTagTimer.hasElapsed(AlignToBranchConstants.DONT_SEE_TAG_WAIT_TIME) ||
        stopTimer.hasElapsed(AlignToBranchConstants
        .POSE_VALIDATION_TIME);
  }
}