// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pooper extends SubsystemBase {
  private final TalonFX pooperMotor = new TalonFX(14, "drivecan");

  /** Creates a new Pooper. */
  public Pooper() {
    final TalonFXConfiguration pooperConfig = new TalonFXConfiguration();

    pooperConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pooperConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    pooperConfig.CurrentLimits.SupplyCurrentLimit = 60;
    //pooperConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    pooperMotor.getConfigurator().apply(pooperConfig);
  }

  public void runPooper(double speed) {
    pooperMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
