// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.urcl.URCL;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;
  private final boolean kUseLimelight = true;
  private final Field2d field;

  public Robot() {
    m_robotContainer = new RobotContainer();
    field = new Field2d();
    SmartDashboard.putData("Field", field);
  }

  @Override
  public void robotInit() {
    URCL.start(DataLogManager.getLog());
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    
    if (kUseLimelight) {
      updateVisionPose();
    }

    updatePose(RobotContainer.drivetrain.getState().Pose);
  }

  private void updateVisionPose() {
    var driveState = m_robotContainer.drivetrain.getState();
    double headingDeg = driveState.Pose.getRotation().getDegrees();
    double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);
    
    LimelightHelpers.SetRobotOrientation("limelight-front", headingDeg, 0, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation("limelight-back", headingDeg, 0, 0, 0, 0, 0);
    
    var llMeasurement1 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-front");
    var llMeasurement2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-back");
    
    Pose2d fusedPose = null;
    double bestTimestamp = 0;
    int validSources = 0;
    
    if (llMeasurement1 != null && llMeasurement1.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
        fusedPose = llMeasurement1.pose;
        bestTimestamp = llMeasurement1.timestampSeconds;
        validSources++;
    }
    
    if (llMeasurement2 != null && llMeasurement2.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
        if (validSources > 0) {
            fusedPose = new Pose2d(
                (fusedPose.getX() + llMeasurement2.pose.getX()) / 2.0,
                (fusedPose.getY() + llMeasurement2.pose.getY()) / 2.0,
                fusedPose.getRotation()
            );
            bestTimestamp = Math.max(bestTimestamp, llMeasurement2.timestampSeconds);
        } else {
            fusedPose = llMeasurement2.pose;
            bestTimestamp = llMeasurement2.timestampSeconds;
        }
        validSources++;
    }
    
    if (validSources > 0) {
        double visionStdDev = 0.5 / Math.sqrt(validSources);
        m_robotContainer.drivetrain.addVisionMeasurement(fusedPose, bestTimestamp, visionStdDev);
    }

    SmartDashboard.putNumber("Limelight Front X", llMeasurement1 != null ? llMeasurement1.pose.getX() : -1);
    SmartDashboard.putNumber("Limelight Front Y", llMeasurement1 != null ? llMeasurement1.pose.getY() : -1);
    SmartDashboard.putNumber("Limelight Front Tags", llMeasurement1 != null ? llMeasurement1.tagCount : 0);
    SmartDashboard.putNumber("Limelight Back X", llMeasurement2 != null ? llMeasurement2.pose.getX() : -1);
    SmartDashboard.putNumber("Limelight Back Y", llMeasurement2 != null ? llMeasurement2.pose.getY() : -1);
    SmartDashboard.putNumber("Limelight Back Tags", llMeasurement2 != null ? llMeasurement2.tagCount : 0);
  }

  public void updatePose(Pose2d pose){
    field.setRobotPose(pose);
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }
}
