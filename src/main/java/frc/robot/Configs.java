
package frc.robot;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

public final class Configs {
  
  public static final class ArmivatorSubsystem {
    public static final SparkFlexConfig armConfig = new SparkFlexConfig();
    public static final AbsoluteEncoderConfig armEncoderConfig = new AbsoluteEncoderConfig();
    public static final SparkFlexConfig elevatorConfig = new SparkFlexConfig();
    public static final SparkFlexConfig intakeConfig = new SparkFlexConfig();
    public static final SparkFlexConfig climberConfig = new SparkFlexConfig();
    public static final SparkFlexConfig hopperConfig = new SparkFlexConfig();

    static {
      // Configure Absolute encoder for arm
      armEncoderConfig.inverted(true).positionConversionFactor(360);

      // Configure basic settings of the arm motor
      armConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(40).voltageCompensation(12).apply(armEncoderConfig);
      
      /*
       * Configure the closed loop controller. We want to make sure we set the
       * feedback sensor as the primary encoder.
       */
      armConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
          // Set PID values for position control
          .p(.2)
          .outputRange(-1, 1)
          .maxMotion
          // Set MAXMotion parameters for position control
          .maxVelocity(2000)
          .maxAcceleration(10000)
          .allowedClosedLoopError(0.0025);

      // Configure basic settings of the elevator motor
      elevatorConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(50).voltageCompensation(12).inverted(true);

      /*
       * Configure the closed loop controller. We want to make sure we set the
       * feedback sensor as the primary encoder.
       */
      elevatorConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // Set PID values for position control
          .p(0.1)
          .outputRange(-1, 1)
          .maxMotion
          // Set MAXMotion parameters for position control
          .maxVelocity(4200)
          .maxAcceleration(6000)
          .allowedClosedLoopError(0.5);


      intakeConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(100);
      

      climberConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(100);
      
      hopperConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(100);
        
      
    }
  }
}
