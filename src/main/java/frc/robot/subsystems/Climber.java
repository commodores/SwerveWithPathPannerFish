package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Climber extends SubsystemBase {
    private final TalonFX climberMotor = new TalonFX(13, "drivecan");
    private final Servo climberServo = new Servo(9);
    private boolean locked = false;

    // Constants
    private static final double GEAR_RATIO = 64.0 * 3.5;  // 224:1 total
    private static final double DEGREES_PER_ROTATION = 360.0 / GEAR_RATIO;  // 1.6071 degrees per rotation

    private static final double START_ANGLE = 7.0;     // Initial angle from parallel
    private static final double GRAB_ANGLE = 90.0;     // Perpendicular to floor
    private static final double CLIMB_ANGLE = -10.0;   // 100 degrees past perpendicular

    public Climber() {
        final TalonFXConfiguration climberConfig = new TalonFXConfiguration();

        climberConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        climberConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        climberConfig.CurrentLimits.SupplyCurrentLimit = 60;

        climberMotor.getConfigurator().apply(climberConfig);

        resetClimbEncoder();
        unLockClimber();
    } 

    public void resetClimbEncoder() {
        climberMotor.setPosition(degreesToRotations(START_ANGLE)); // Set encoder to match real-world start angle
    }

    public double getClimberPositionDegrees() {
        return rotationsToDegrees(climberMotor.getPosition().getValueAsDouble());
    }

    public void climberForward() {
        if (!locked && getClimberPositionDegrees() < GRAB_ANGLE) {
            climberMotor.set(-0.5);
        } else {
            stopClimber();
        }
    }

    public void climberBack() {
        if (getClimberPositionDegrees() > CLIMB_ANGLE) {
            climberMotor.set(0.372);
        } else {
            stopClimber();
        }
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

    @Override
    public void periodic() {
        // Debugging output
        SmartDashboard.putNumber("Climber Position: ", getClimberPositionDegrees());
    }

    // ** Conversion Functions **
    private double rotationsToDegrees(double rotations) {
        return rotations * DEGREES_PER_ROTATION;
    }

    private double degreesToRotations(double degrees) {
        return degrees / DEGREES_PER_ROTATION;
    }
}
