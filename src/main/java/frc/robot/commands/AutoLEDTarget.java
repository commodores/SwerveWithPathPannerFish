// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CANdleSubsystem;

public class AutoLEDTarget extends Command {

  private final CANdleSubsystem m_CANdle;

  private static final String LIMELIGHT_NAME = "limelight";
  
  // Corrected Tolerances
  private static final double RY_TOLERANCE = 1;
  private static final double TX_TOLERANCE = 0.06;
  private static final double TZ_TOLERANCE = 0.04;

  double error;
  double tx;
  double tz;
  double ry;

  Optional<Alliance> ally;

  /** Creates a new AutoLEDTarget. */
  public AutoLEDTarget(CANdleSubsystem CANdle) {
    m_CANdle = CANdle;
    addRequirements(m_CANdle);
  }

  @Override
  public void initialize() {
    ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        m_CANdle.setColor(255, 0, 0);
      } else if (ally.get() == Alliance.Blue) {
        m_CANdle.setColor(0, 0, 255);
      }
    } else {
      m_CANdle.setColor(200, 200, 0);
    }
  }

  @Override
  public void execute() {
    double[] positions = LimelightHelpers.getBotPose_TargetSpace(LIMELIGHT_NAME);
    tx = positions[0];
    tz = positions[2];
    ry = positions[4];

    // Check if values are within tolerance
    if (LimelightHelpers.getTV(LIMELIGHT_NAME)) {
      if ((Math.abs(tx - 0.19) <= TX_TOLERANCE || Math.abs(tx + 0.19) <= TX_TOLERANCE)&&
          Math.abs(tz + 0.52) <= TZ_TOLERANCE &&
          Math.abs(ry) <= RY_TOLERANCE) {
        m_CANdle.setColor(0, 255, 0); // Green if within tolerance
      } else {
        if (ally.isPresent()) {
          if (ally.get() == Alliance.Red) {
            m_CANdle.setColor(255, 0, 0);
          } else if (ally.get() == Alliance.Blue) {
            m_CANdle.setColor(0, 0, 255);
          }
        }
      }
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
