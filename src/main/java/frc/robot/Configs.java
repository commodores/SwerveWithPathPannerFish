package frc.robot;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import com.revrobotics.spark.config.SparkFlexConfig;

public final class Configs {
  
  public static final class ArmivatorSubsystem {
    public static final SparkFlexConfig armConfig = new SparkFlexConfig();
    public static final SparkFlexConfig elevatorConfig = new SparkFlexConfig();   

    static {
      armConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(60)
            .inverted(false);

      armConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingMinInput(0)
        .positionWrappingMaxInput(360)
        .p(0.015)
        //.d(0.01)
        //.i(0.0001)
        .minOutput(-.75)
        .maxOutput(.75);

      armConfig
        .signals
        .absoluteEncoderPositionPeriodMs(20)
        .absoluteEncoderVelocityPeriodMs(20);

      armConfig
        .absoluteEncoder
        .inverted(true)
        .positionConversionFactor(360.0)
        .velocityConversionFactor(360.0 / 60);

      elevatorConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(60)
        .inverted(true)
        .softLimit
        .reverseSoftLimit(0)
        .reverseSoftLimitEnabled(true)
        .forwardSoftLimit(27)
        .forwardSoftLimitEnabled(true);

      elevatorConfig
        .encoder
        .positionConversionFactor(1.0998);//4 to 1 gear ratio .8755 radius spool
      
      elevatorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(.15)
        .minOutput(-.55)//-.45
        .maxOutput(.65); //.45     
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

  public static final class Level1Subsystem {
    public static final SparkFlexConfig level1ArmConfig = new SparkFlexConfig();
    public static final SparkFlexConfig level1IntakeConfig = new SparkFlexConfig();   

    static {
      level1ArmConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(60)
            .inverted(false);

      level1ArmConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingMinInput(0)
        .positionWrappingMaxInput(360)
        .p(0.015)
        //.d(0.01)
        //.i(0.0001)
        .minOutput(-.75)
        .maxOutput(.75);

      level1ArmConfig
        .signals
        .absoluteEncoderPositionPeriodMs(20)
        .absoluteEncoderVelocityPeriodMs(20);

      level1ArmConfig
        .encoder
        .positionConversionFactor(360.0 / 15.0)
        .velocityConversionFactor((360.0 / 15.0) / 60);

      level1IntakeConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(60);
    }
  }
}