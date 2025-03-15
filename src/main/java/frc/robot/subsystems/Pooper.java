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
import frc.robot.Constants.PooperArmSetpoints;
import frc.robot.Constants.PooperConstants;

public class Pooper extends SubsystemBase {
    private final SparkFlex m_pooperArmMotor;
    private final RelativeEncoder m_encoder;
    private final SparkClosedLoopController m_sparkPidController;
    private final ProfiledPIDController m_profilePID;
    private final ArmFeedforward feedforward;
    private static final double ALLOWABLE_ERROR = 1; //TODO change allowable error to make it more accurate or to make scoring faster
    private double m_armGoalAngle = PooperArmSetpoints.kStow;
    private double setpoint;//0 stow, 1 floor, 2 score)

    public Pooper() {
        m_pooperArmMotor = new SparkFlex(PooperConstants.pooperArmMotor, MotorType.kBrushless);
        m_sparkPidController = m_pooperArmMotor.getClosedLoopController();
        m_encoder = m_pooperArmMotor.getEncoder();

        m_pooperArmMotor.configure(
        Configs.PooperSubsystem.pooperArmConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

        m_profilePID = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(360, 1200));
        m_profilePID.enableContinuousInput(0, 360);
        m_profilePID.setTolerance(ALLOWABLE_ERROR);
        
        // Create a new ArmFeedforward with gains kS, kG, kV, and kA
        feedforward = new ArmFeedforward(0.001, 1.1, 0.25, 0.04);

        setpoint = 0;

        m_encoder.setPosition(135);
        
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

        SmartDashboard.putNumber("Pooper Arm Angle", getAngle());
        SmartDashboard.putNumber(" Pooper Arm Velocity", getVelocity());
        SmartDashboard.putNumber("Pooper Arm Goal Angle", m_armGoalAngle);
        SmartDashboard.putBoolean("Pooper Arm At Goal", isArmAtGoal());
        SmartDashboard.putNumber("Pooper Arm Setpoint", setpoint);
    }


    public boolean isArmAtGoal() {
        double error = m_armGoalAngle - getAngle();
        return Math.abs(error) < ALLOWABLE_ERROR;
    }

    public double getArmGoalAngle() {
        return m_armGoalAngle;
    }

    public double getSpeed() {
        return m_pooperArmMotor.getAppliedOutput();
    }

    public void stop() {
        m_pooperArmMotor.set(0);
    }

    public void setSpeed(double speed) {
        m_pooperArmMotor.set(speed);
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
        return (Math.abs(this.getAngle() - this.getArmGoalAngle()) <= Pooper.ALLOWABLE_ERROR);
    }

    public double getArmSetpoint() {
        return setpoint;
    }

    public void setArmSetpoint(double setSetpoint) {
        setpoint = setSetpoint;
    }

   


}