
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
      armConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12);
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
          .p(.004)//.004//.005
          //.i(.000003) //.0000001
          //.d(.018)
          .positionWrappingEnabled(true)
          .positionWrappingInputRange(0, 2*Math.PI)
          .outputRange(-1, 1)
          .maxMotion
          // Set MAXMotion parameters for position control
          .maxVelocity(1000)
          .maxAcceleration(6000)//10000
          .allowedClosedLoopError(.01);

      // Configure basic settings of the elevator motor
      elevatorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12).inverted(true);
      /*
       * Configure the closed loop controller. We want to make sure we set the
       * feedback sensor as the primary encoder.
       */
      elevatorConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // Set PID values for position control
          .p(0.0125)//0.0125
          //.d(.04)
          
          .outputRange(-1, 1)
          .maxMotion
          // Set MAXMotion parameters for position control
          .maxVelocity(2200)//4200
          .maxAcceleration(6000)//6000
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
  
  
