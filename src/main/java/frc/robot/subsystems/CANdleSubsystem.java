// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;

public class CANdleSubsystem extends SubsystemBase {
  private final CANdle candle;
  /** Creates a new CANdleSub. */
  public CANdleSubsystem() {
    candle = new CANdle(21, "drivecan");
    CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.GRB;
        configAll.brightnessScalar = 0.1;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        candle.configAllSettings(configAll, 100);

  }

  public void setColor(int r, int g, int b){
    candle.setLEDs(r, g, b,0,0,30);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}