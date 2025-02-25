// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
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

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmivatorConstants;

public class Arm extends SubsystemBase {
    private final SparkFlex m_armMotor;
    private final AbsoluteEncoder m_absoluteEncoder;

    private static final double PIVOT_ERROR = 3;
    private static final double GEAR_RATIO = 1;

    private final SparkClosedLoopController m_sparkPidController;

    private final ProfiledPIDController m_profilePID;

    private final ArmFeedforward feedforward;
    private static final double ALLOWABLE_ERROR = .5; //TODO change allowable error to make it more accurate or to make scoring faster
    private double m_armGoalAngle = Double.MIN_VALUE;

    public Arm() {
        m_armMotor = new SparkFlex(ArmivatorConstants.armMotor, MotorType.kBrushless);
        m_absoluteEncoder = m_armMotor.getAbsoluteEncoder();        
        m_sparkPidController = m_armMotor.getClosedLoopController();

        SparkMaxConfig armConfig = new SparkMaxConfig();

        armConfig.idleMode(IdleMode.kBrake)
            .smartCurrentLimit(60)
            .inverted(true)
            .softLimit
            .reverseSoftLimit(.441)
            .reverseSoftLimitEnabled(true)
            .forwardSoftLimit(4)
            .forwardSoftLimitEnabled(true);
        
        armConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .positionWrappingEnabled(true)
            .positionWrappingMinInput(0)
            .positionWrappingMaxInput(360)
            .p(0.001)
            .minOutput(-.5)
            .maxOutput(.5);
        
        armConfig.signals
            .absoluteEncoderPositionPeriodMs(20)
            .absoluteEncoderVelocityPeriodMs(20);
        
        armConfig.absoluteEncoder
            .inverted(true)
            .positionConversionFactor(360.0 / GEAR_RATIO)
            .velocityConversionFactor(360.0 / GEAR_RATIO / 60);
        
        m_armMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);       
        
        
        m_profilePID = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(360, 1200));
        m_profilePID.enableContinuousInput(0, 360);
        
        // Create a new ArmFeedforward with gains kS, kG, kV, and kA
        //feedforward = new ArmFeedforward(0.1, 1.44, 0.41, 0.06);
        feedforward = new ArmFeedforward(0, 0, 0, 0);

    }

    public void moveArmToAngle(double goal) {
        m_armGoalAngle = goal;
        double currentAngle = getRelativeAngle();
        m_profilePID.calculate(currentAngle, goal);
        TrapezoidProfile.State setpoint = m_profilePID.getSetpoint();

        //double feedForwardVolts = m_wpiFeedForward.calculateWithVelocities(currentAngle, m_relativeEncoder.getVelocity(), setpoint.velocity);
        double feedForwardVolts = feedforward.calculate(Math.toRadians(currentAngle), Math.toRadians(setpoint.velocity));

        m_sparkPidController.setReference(setpoint.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedForwardVolts);
        SmartDashboard.putNumber("feedForwardVolts", feedForwardVolts);
    }
    

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Angle", getRelativeAngle());
        SmartDashboard.putNumber("Arm Velocity", getRelativeVelocity());
        SmartDashboard.putNumber("Arm Goal Angle", m_armGoalAngle);
    }


    public boolean isArmAtGoal() {
        double error = m_armGoalAngle - getRelativeAngle();
        return Math.abs(error) < ALLOWABLE_ERROR;
    }

    public double getArmGoalAngle() {
        return m_armGoalAngle;
    }

    public double getSpeed() {
        return m_armMotor.getAppliedOutput();
    }

    public void stop() {
        m_armMotor.set(0);
    }

    public void setSpeed(double speed) {
        m_armMotor.set(speed);
    }

    public double getRelativeAngle() {
        return m_absoluteEncoder.getPosition();
    }

    public double getRelativeVelocity() {
        return m_absoluteEncoder.getVelocity();
    }

    public double getAbsoluteAngle() {
        return m_absoluteEncoder.getPosition();
    }

    private void resetPidController() {
        m_profilePID.reset(getRelativeAngle(), getRelativeVelocity());
    }

    public boolean isAtGoalAngle() {
        return (Math.abs(this.getRelativeAngle() - this.getArmGoalAngle()) <= Arm.PIVOT_ERROR);
    }

    public Command createMoveArmtoAngleCommand(Double angle) {
        return createResetPidControllerCommand().andThen(runEnd(() -> moveArmToAngle(angle), this::stop)).withName("Go to angle" + angle);
    }

    private Command createResetPidControllerCommand() {
        return runOnce(this::resetPidController);
    }


}