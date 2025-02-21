
package frc.robot;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.util.Units;

import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

public final class Configs {
  
  public static final class ArmivatorSubsystem {
    public static final SparkFlexConfig armConfig = new SparkFlexConfig();
    public static final AbsoluteEncoderConfig armEncoderConfig = new AbsoluteEncoderConfig();
    public static final SparkFlexConfig elevatorConfig = new SparkFlexConfig();
   

    static {
      // Configure basic settings of the arm motor
      armConfig.idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40)
        .voltageCompensation(12)
        .softLimit
        .reverseSoftLimit(.441)
        .reverseSoftLimitEnabled(true)
        .forwardSoftLimit(3.74)
        .forwardSoftLimitEnabled(true);
      
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
          .p(.004)
          .positionWrappingEnabled(true)
          .positionWrappingInputRange(0, 2*Math.PI)
          .outputRange(-1, 1)
          .maxMotion
          // Set MAXMotion parameters for position control
          .maxVelocity(2500)
          .maxAcceleration(250)
          .allowedClosedLoopError(.01);

      // Configure basic settings of the elevator motor
      elevatorConfig.idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40)
        .voltageCompensation(12)
        .inverted(true)
        .softLimit
        .reverseSoftLimit(0)
        .reverseSoftLimitEnabled(true)
        .forwardSoftLimit(96.8)
        .forwardSoftLimitEnabled(true);;
      /*
       * Configure the closed loop controller. We want to make sure we set the
       * feedback sensor as the primary encoder.
       */
      elevatorConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // Set PID values for position control
          .p(0.0125)          
          .outputRange(-1, 1)
          .maxMotion
          // Set MAXMotion parameters for position control
          .maxVelocity(2000)//4200
          .maxAcceleration(200)//6000
          .allowedClosedLoopError(.5);
    }
  }

  public static final class IntakeSubsytem {

      public static final SparkFlexConfig intakeConfig = new SparkFlexConfig();
      public static final SparkFlexConfig hopperConfig = new SparkFlexConfig();

      static{
            intakeConfig
              .idleMode(IdleMode.kBrake)
              .smartCurrentLimit(50);

            hopperConfig
              .idleMode(IdleMode.kBrake)
              .smartCurrentLimit(50);
            }
        }
     }
  
  
