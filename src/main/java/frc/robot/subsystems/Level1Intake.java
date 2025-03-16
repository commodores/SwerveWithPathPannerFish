package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.Level1Constants;

public class Level1Intake extends SubsystemBase {
    private final SparkFlex m_level1IntakeMotor;
    
    public Level1Intake() {
        m_level1IntakeMotor = new SparkFlex(Level1Constants.level1IntakeMotor, MotorType.kBrushless);
        
        m_level1IntakeMotor.configure(
            Configs.Level1Subsystem.level1IntakeConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    /** Sets the intake speed, ensuring it stays within safe limits */
    public void setIntakeSpeed(double speed) {
        double clampedSpeed = Math.max(-1.0, Math.min(1.0, speed)); // Limit to [-1, 1]
        m_level1IntakeMotor.set(clampedSpeed);
    }

    /** Stops the intake motor */
    public void stop() {
        m_level1IntakeMotor.set(0);
    }

    /** Returns the current motor draw in Amps */
    public double getMotorCurrent() {
        return m_level1IntakeMotor.getOutputCurrent();
    }

    @Override
    public void periodic() {
        // Optional: Log motor speed for debugging
        System.out.println("Intake Speed: " + m_level1IntakeMotor.get());
    }
}
