// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants;

public class TestArm extends TrapezoidProfileSubsystem {

  // Define the motor and encoders
  private final SparkFlex shoulderMotor;
  private final PIDController m_PIDController;
  private final RelativeEncoder m_relative_encoder;
  private final AbsoluteEncoder m_absolute_encoder;

  private final ArmFeedforward m_feedforward =
      new ArmFeedforward(
          0,0,0,0);

  /** Creates a new ArmShoulder. */
  public TestArm() {
    super(
        // The constraints for the generated profiles
        new TrapezoidProfile.Constraints(0, 0),
        // The initial position of the mechanism
        Math.PI);

    shoulderMotor = new SparkFlex(5, MotorType.kBrushless);
    
    shoulderMotor.restoreFactoryDefaults();
    shoulderMotor.setInverted(false);
    shoulderMotor.setSmartCurrentLimit(1);
    shoulderMotor.setIdleMode(IdleMode.kBrake);

    m_PIDController = shoulderMotor.getPIDController();
    m_PIDController.setP(Constants.ShoulderConstants.kP);
    m_PIDController.setI(Constants.ShoulderConstants.kI);
    m_PIDController.setD(Constants.ShoulderConstants.kD);
    m_PIDController.setFF(Constants.ShoulderConstants.kFF);

    m_relative_encoder = shoulderMotor.getEncoder();
    m_relative_encoder.setPositionConversionFactor((2 * Math.PI) / Constants.ShoulderConstants.kGearRatio); //Converted to Radians

    m_absolute_encoder = shoulderMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    m_absolute_encoder.setPositionConversionFactor(Math.PI * 2); //Converted to Radians?

    m_PIDController.setFeedbackDevice(m_relative_encoder);
    
    syncEncoder();

  }

  @Override
  public void periodic() {
    double relativeEncoderValue = m_relative_encoder.getPosition();
    double absoluteEncoderValue = m_absolute_encoder.getPosition();
    //syncEncoder();
    
    // Display current values on the SmartDashboard
    //SmartDashboard.putNumber("arm Output", shoulderMotor.getAppliedOutput());
    SmartDashboard.putNumber("Shoulder Relative Encoder Degrees", Units.radiansToDegrees(relativeEncoderValue));
    SmartDashboard.putNumber("Shoulder Absolute Encoder Degrees", Units.radiansToDegrees(absoluteEncoderValue));
    SmartDashboard.putNumber("Shoulder Relative Encoder Radians", relativeEncoderValue);
    SmartDashboard.putNumber("Shoulder Absolute Encoder Radians", absoluteEncoderValue);

    SmartDashboard.putNumber("Shoulder WAWA", getEncoder());

    // Execute the super class periodic method
    super.periodic();
  }

  @Override
  protected void useState(TrapezoidProfile.State setPoint) {
    // Calculate the feedforward fromteh setPoint
    double feedforward = m_feedforward.calculate(setPoint.position, setPoint.velocity);

    // Add the feedforward to the PID output to get the motor output
    // The ArmFeedForward computes in radians. We need to convert back to degrees.
    // Remember that the encoder was already set to account for the gear ratios.
    
    m_PIDController.setReference(setPoint.position, CANSparkMax.ControlType.kPosition, 0);
    
    //SmartDashboard.putNumber("Shoulder Feedforward", feedforward);
    //SmartDashboard.putNumber("Shoulder SetPoint", Units.metersToInches(setPoint.position));
    //SmartDashboard.putNumber("Shoulder Velocity", Units.metersToInches(setPoint.velocity));
  }

  public Command setArmGoalCommand(double kArmOffsetRads) {
    return Commands.runOnce(() -> setGoal(kArmOffsetRads), this);
  }

  public void syncEncoder(){
    double position = m_absolute_encoder.getPosition()+Constants.ShoulderConstants.kShoulderOffset;
    m_relative_encoder.setPosition(position);
  }

  public double getEncoder(){
    return m_relative_encoder.getPosition();
  }
}