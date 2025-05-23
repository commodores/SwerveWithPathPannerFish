// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import com.ctre.phoenix6.Utils;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private final boolean kUseLimelight = false;

  private final Field2d field;

  public Robot() {
    m_robotContainer = new RobotContainer();
    
    field = new Field2d();

    SmartDashboard.putData("Field", field);

  }

  @Override
  public void robotInit() {
    
  } 

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    
    if (kUseLimelight) 
    {
      var driveState = RobotContainer.drivetrain.getState();
      double headingDeg = driveState.Pose.getRotation().getDegrees();
      double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

      // Limelight names
      String[] limelights = {"limelight", "limelight-back"} ;

      // Place to store standard deviations of the limelights' readings
      double limelightAvgStdDev[] = new double[limelights.length];
      
      // Constants for dynamic standard deviation threshold
      double baseStdDev = 0.2; // Base standard deviation at close range
      double k_d = 0.05;       // Scaling factor for distance
      double k_t = 0.1;        // Reward for additional tags

      // Process each Limelight
      for (int i = 0; i < limelights.length; i++) 
      {
        String limelightName = limelights[i];
        LimelightHelpers.SetRobotOrientation(limelightName, headingDeg, 0, 0, 0, 0, 0);

        // Calculate the average standard deviation for this Limelight
        double[] stddevs = NetworkTableInstance.getDefault()
          .getTable(limelightName)
          .getEntry("stddevs")
          .getDoubleArray(new double[6]);

        limelightAvgStdDev[i] = (stddevs[0] + stddevs[1] + stddevs[5]) / 3;

        // Retrieve vision measurement from the Limelight
        var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

        if (llMeasurement != null && llMeasurement.tagCount > 0) 
        {
          // Calculate dynamic standard deviation threshold
          double dynamicThreshold = baseStdDev + (k_d * llMeasurement.avgTagDist) - (k_t * llMeasurement.tagCount);

          // Check conditions: dynamic threshold and angular velocity
          if (limelightAvgStdDev[i] < dynamicThreshold && Math.abs(omegaRps) < 2.0) 
          {
            LimelightHelpers.SetRobotOrientation(limelightName, headingDeg, 0, 0, 0, 0, 0);
            RobotContainer.drivetrain.addVisionMeasurement
            (
              llMeasurement.pose, 
              Utils.fpgaToCurrentTime(llMeasurement.timestampSeconds)
            );
          }
        }
      }
    }
  

    updatePose(RobotContainer.drivetrain.getState().Pose);
  }

  public void updatePose(Pose2d pose){
    field.setRobotPose(pose);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
