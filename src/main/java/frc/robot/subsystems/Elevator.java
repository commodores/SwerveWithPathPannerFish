// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmivatorConstants;

import com.playingwithfusion.TimeOfFlight;

import com.revrobotics.RelativeEncoder;

public class Elevator extends SubsystemBase {

    public static final double ELEVATOR_ERROR = Units.inchesToMeters(3);

    private final SparkFlex m_elevatorMotor;    
    private final RelativeEncoder m_encoder;
    private TimeOfFlight m_elevatorSensor;

    private final ProfiledPIDController m_profilePID;
    private final SparkClosedLoopController m_sparkPidController;
    private final ElevatorFeedforward feedforward;

    private double m_goalHeight;


    public Elevator() {

        m_elevatorMotor = new SparkFlex(ArmivatorConstants.elevatorMotor, MotorType.kBrushless);
        m_encoder = m_elevatorMotor.getEncoder();
        m_elevatorSensor = new TimeOfFlight(4);

        SparkMaxConfig elevatorConfig = new SparkMaxConfig();
        
        elevatorConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(60)
          .inverted(true)
          .softLimit
          .reverseSoftLimit(0)
          .reverseSoftLimitEnabled(true);

        elevatorConfig
          .absoluteEncoder
          .positionConversionFactor(1.375);//4 to 1 gear ratio .8755 radius spool
        
        elevatorConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .p(.001)
          .minOutput(-.5)
          .maxOutput(.5);

        m_sparkPidController = m_elevatorMotor.getClosedLoopController();
        elevatorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

        m_profilePID = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(270, 900));

        // Create a new ElevatorFeedforward with gains kS, kG, kV, and kA
        feedforward = new ElevatorFeedforward(0.01, 0.36, 3.13, 0.05);

        m_elevatorMotor.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    public double getHeight() {
        return m_encoder.getPosition();
    }

    public void setSpeed(double speed) {
        m_elevatorMotor.set(speed);
    }

    public double getElevatorDistanceInInch(){

      return (m_elevatorSensor.getRange()*0.03937008)-2;
          
    }

    public boolean isAtBottom() {
        return getElevatorDistanceInInch()<.1;
    }

    public boolean isAtTop() {
        return getElevatorDistanceInInch()>=20;
    }

    public void stop() {
        m_elevatorMotor.set(0);
    }

    public double getGoalHeight() {
        return m_goalHeight;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Height", getHeight());
        SmartDashboard.putNumber("Elevator Distance", getElevatorDistanceInInch());
        SmartDashboard.putBoolean("Elevator at bottom", isAtBottom());
        SmartDashboard.putBoolean("Elevator at top", isAtTop());
        SmartDashboard.putNumber("Elevator Goal Height", m_goalHeight);
    }

    public double getEncoderVel() {
        return m_encoder.getVelocity();
    }

    public void goToHeight(double goalHeight) {
        m_profilePID.calculate(getHeight(), goalHeight);
        m_goalHeight = goalHeight;
        TrapezoidProfile.State setpoint = m_profilePID.getSetpoint();

        double feedForwardVolts = feedforward.calculateWithVelocities(
            getEncoderVel(),
            setpoint.velocity);

        m_sparkPidController.setReference(setpoint.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedForwardVolts);
        SmartDashboard.putNumber("feedForwardVolts", feedForwardVolts);

    }

    public void resetPidController() {
        m_profilePID.reset(getHeight(), getEncoderVel());
    }

    public boolean isAtGoalHeight() {
        return Math.abs(getHeight() - m_goalHeight) <= ELEVATOR_ERROR;
    }

    //command factories//
    public Command createResetPidControllerCommand() {
        return runOnce(this::resetPidController);
    }

    public Command createMoveElevatorToHeightCommand(double height) {
        return createResetPidControllerCommand().andThen(
        runEnd(() -> goToHeight(height), m_elevatorMotor::stopMotor)).withName("Elevator go to height" + height);
    }


}