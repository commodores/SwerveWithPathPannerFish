// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlignToCoralStationConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AlignToCoralStation extends Command {
  private PIDController xController, yController, rotController;
  private Timer dontSeeTagTimer, stopTimer, totalTimer;
  private CommandSwerveDrivetrain drivebase;
  private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();
  private double tagID = -1;

  public AlignToCoralStation(CommandSwerveDrivetrain drivebase) {
    xController = new PIDController(AlignToCoralStationConstants.X_CORALSTATION_ALIGNMENT_P, 0.0, 0);     // Move towards reef
    yController = new PIDController(AlignToCoralStationConstants.Y_CORALSTATION_ALIGNMENT_P, 0.0, 0);     // Strafe to branch
    rotController = new PIDController(AlignToCoralStationConstants.ROT_CORALSTATION_ALIGNMENT_P, 0, 0);   // Rotation towards target
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
    rotController.setSetpoint(AlignToCoralStationConstants.ROT_SETPOINT_CORALSTATION_ALIGNMENT);
    rotController.setTolerance(AlignToCoralStationConstants.ROT_TOLERANCE_CORALSTATION_ALIGNMENT);
    // Move towards CORALSTATION
    xController.setSetpoint(AlignToCoralStationConstants.X_SETPOINT_CORALSTATION_ALIGNMENT); 
    xController.setTolerance(AlignToCoralStationConstants.X_TOLERANCE_CORALSTATION_ALIGNMENT);
    // Strafe to branch
    yController.setSetpoint(AlignToCoralStationConstants.Y_SETPOINT_CORALSTATION_ALIGNMENT); 
    yController.setTolerance(AlignToCoralStationConstants.Y_TOLERANCE_CORALSTATION_ALIGNMENT);

    tagID = LimelightHelpers.getFiducialID("limelight-back");
  }

  @Override
  public void execute() {
    if (LimelightHelpers.getTV("limelight-back") && LimelightHelpers.getFiducialID("limelight-back") == tagID) {
      this.dontSeeTagTimer.reset();

      double[] postions = LimelightHelpers.getBotPose_TargetSpace("limelight-back");
      SmartDashboard.putNumber("tx", postions[0]);
      SmartDashboard.putNumber("tz", postions[2]);
      SmartDashboard.putNumber("ry", postions[4]);

      double xSpeed = -xController.calculate(postions[2]);       // Move towards reef
      double ySpeed = yController.calculate(postions[0]);      // Strafe to branch
      double rotValue = rotController.calculate(postions[4]);  // Rotation towards target

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
    return this.dontSeeTagTimer.hasElapsed(AlignToCoralStationConstants.DONT_SEE_TAG_WAIT_TIME) ||
        stopTimer.hasElapsed(AlignToCoralStationConstants.POSE_VALIDATION_TIME) ||
        totalTimer.hasElapsed(AlignToCoralStationConstants.POSE_TOTAL_TIME);
  }
}