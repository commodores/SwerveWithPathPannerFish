
package frc.robot;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

public final class Configs {
  
  public static final class ArmivatorSubsystem {
    public static final SparkFlexConfig armConfig = new SparkFlexConfig();
    public static final AbsoluteEncoderConfig armEncoderConfig = new AbsoluteEncoderConfig();
    public static final SparkFlexConfig elevatorConfig = new SparkFlexConfig();
    public static final SparkFlexConfig intakeConfig = new SparkFlexConfig();
    public static final TalonFXConfiguration climberConfig = new TalonFXConfiguration();
    public static final SparkFlexConfig hopperConfig = new SparkFlexConfig();

    static {
      // Configure Absolute encoder for arm
     // armEncoderConfig.inverted(true).positionConversionFactor(2.0 * Math.PI);

      // Configure basic settings of the arm motor
      armConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(40).voltageCompensation(12);//apply(armEncoderConfig)
      armConfig.absoluteEncoder
        .inverted(true)
        .positionConversionFactor(2*Math.PI);
      /*
       * Configure the closed loop controller. We want to make sure we set the
       * feedback sensor as the primary encoder.
       */
      armConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
          // Set PID values for position control
          //.p(.01)
          .pidf(.01, 0, 0, .02)
          .positionWrappingEnabled(true)
          .positionWrappingInputRange(0, 2*Math.PI)
          .outputRange(-1, 1)
          .maxMotion
          // Set MAXMotion parameters for position control
          .maxVelocity(2000)
          .maxAcceleration(10000)
          .allowedClosedLoopError(.05);

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
          .p(0.01)
          .outputRange(-1, 1)
          .maxMotion
          // Set MAXMotion parameters for position control
          .maxVelocity(4200)
          .maxAcceleration(6000)
          .allowedClosedLoopError(2.0);


      intakeConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(50);
      

      //  climberConfig
        //.setNeutralMode(NeutralModeValue.Brake)
      
        
    
      // climberConfig does not support smartCurrentLimit
      // climberConfig.smartCurrentLimit(50);
      
      hopperConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(50);
        
      
    }
  }
}
