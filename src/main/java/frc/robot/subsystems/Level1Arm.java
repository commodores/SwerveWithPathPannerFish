// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.Level1ArmSetpoints;
import frc.robot.Constants.Level1Constants;

public class Level1Arm extends SubsystemBase {
    private final SparkFlex m_level1ArmMotor;
    private final RelativeEncoder m_encoder;
    private final SparkClosedLoopController m_sparkPidController;
    private final ProfiledPIDController m_profilePID;
    private final ArmFeedforward feedforward;
    private static final double ALLOWABLE_ERROR = 1; //TODO change allowable error to make it more accurate or to make scoring faster
    private double m_armGoalAngle = Level1ArmSetpoints.kStow;
    private double setpoint;//0 stow, 1 floor, 2 score)

    public Level1Arm() {
        m_level1ArmMotor = new SparkFlex(Level1Constants.level1ArmMotor, MotorType.kBrushless);
        m_sparkPidController = m_level1ArmMotor.getClosedLoopController();
        m_encoder = m_level1ArmMotor.getEncoder();

        m_level1ArmMotor.configure(
        Configs.Level1Subsystem.level1ArmConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

        m_profilePID = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(360, 1200));
        m_profilePID.enableContinuousInput(0, 360);
        m_profilePID.setTolerance(ALLOWABLE_ERROR);
        
        // Create a new ArmFeedforward with gains kS, kG, kV, and kA
        feedforward = new ArmFeedforward(0.001, 1.1, 0.25, 0.04);

        setpoint = 0;

        m_encoder.setPosition(128);
        
    }

    public void moveArmToAngle() {
        double currentAngle = getAngle();
        m_profilePID.calculate(currentAngle, m_armGoalAngle);
        TrapezoidProfile.State setpoint = m_profilePID.getSetpoint();

        //double feedForwardVolts = m_wpiFeedForward.calculateWithVelocities(currentAngle, m_relativeEncoder.getVelocity(), setpoint.velocity);
        double feedForwardVolts = feedforward.calculate(Math.toRadians(currentAngle), Math.toRadians(setpoint.velocity));

        m_sparkPidController.setReference(setpoint.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedForwardVolts);
        SmartDashboard.putNumber("feedForwardVolts", feedForwardVolts);
    }

    public void resetAndMoveArmToAngle(double setpoint){
        resetPidController();
        m_armGoalAngle = setpoint;

    }
    

    @Override
    public void periodic() {

        moveArmToAngle();

        SmartDashboard.putNumber("Level1 Arm Angle", getAngle());
        SmartDashboard.putNumber(" Level1 Arm Velocity", getVelocity());
        SmartDashboard.putNumber("Level1 Arm Goal Angle", m_armGoalAngle);
        SmartDashboard.putBoolean("Level1 Arm At Goal", isArmAtGoal());
        SmartDashboard.putNumber("Level1 Arm Setpoint", setpoint);
    }


    public boolean isArmAtGoal() {
        double error = m_armGoalAngle - getAngle();
        return Math.abs(error) < ALLOWABLE_ERROR;
    }

    public double getArmGoalAngle() {
        return m_armGoalAngle;
    }

    public double getSpeed() {
        return m_level1ArmMotor.getAppliedOutput();
    }

    public void stop() {
        m_level1ArmMotor.set(0);
    }

    public void setSpeed(double speed) {
        m_level1ArmMotor.set(speed);
    }

    public double getAngle() {
        return m_encoder.getPosition();
    }

    public double getVelocity() {
        return m_encoder.getVelocity();
    }

    private void resetPidController() {
        m_profilePID.reset(getAngle(), getVelocity());
    }

    public boolean isAtGoalAngle() {
        return (Math.abs(this.getAngle() - this.getArmGoalAngle()) <= Level1Arm.ALLOWABLE_ERROR);
    }

    public double getArmSetpoint() {
        return setpoint;
    }

    public void setArmSetpoint(double setSetpoint) {
        setpoint = setSetpoint;
    }

   


}