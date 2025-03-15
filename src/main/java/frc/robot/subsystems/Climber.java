package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Climber extends SubsystemBase {
    private final TalonFX climberMotor = new TalonFX(ClimberConstants.climberMotor, "drivecan");
    private final Servo climberServo = new Servo(9);
    private boolean locked = false;

    

    public Climber() {
        final TalonFXConfiguration climberConfig = new TalonFXConfiguration();

        climberConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        climberConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        climberConfig.CurrentLimits.SupplyCurrentLimit = 60;
        climberConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        climberMotor.getConfigurator().apply(climberConfig);

        resetClimbEncoder();
        unLockClimber();
    } 

    public void resetClimbEncoder() {
        climberMotor.setPosition(degreesToRotations(ClimberConstants.startAngle)); // Set encoder to match real-world start angle
    }

    public double getClimberPositionDegrees() {
        return rotationsToDegrees(climberMotor.getPosition().getValueAsDouble());
    }

    public void setClimberSpeed(double speed) {
        climberMotor.set(speed);
    }

    public void climberForward() {
        if (!locked) {
            climberMotor.set(0.3);
        } else {
            stopClimber();
        }
    }

    public void climberBack() {
            climberMotor.set(-1.0);
    }

    public void stopClimber() {
        climberMotor.set(0);
    }

    public void lockClimber() {
        climberServo.set(0.5);
        locked = true;
    }

    public void unLockClimber() {
        climberServo.set(0.7);
        locked = false;
    }

    public boolean getIsLocked() {
        return locked;
    }

    @Override
    public void periodic() {
        // Debugging output
        SmartDashboard.putNumber("Climber Position: ", getClimberPositionDegrees());
    }

    // ** Conversion Functions **
    private double rotationsToDegrees(double rotations) {
        return rotations * ClimberConstants.climberDegreesPerRotation;
    }

    private double degreesToRotations(double degrees) {
        return degrees / ClimberConstants.climberDegreesPerRotation;
    }
}
