
package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import  com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import frc.robot.Configs;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
public class Climber extends SubsystemBase {
    private final TalonFX climberMotor = new TalonFX(0);
    
public Climber() {

  // Set your specific configuration parameters here
 
  //climberMotor.configAllSettings(climberConfig);
  //climberMotor.configure()


  //CoreTalonFX m_talonFX;

  //var TalonFXConfigurator = m_talonFX.getConfigurator();


    //configureSparkFlex(climberMotor);
    //climberMotor.configure(
         // Configs.ArmivatorSubsystem.climberConfig,
         // .kResetSafeParameters,
         // .kPersistParameters);
    }
    
    
   /*  private void configureSparkFlex(SparkFlex climberMotor) {
        SparkFlexConfig config = new SparkFlexConfig();
        config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(100);
        climberMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters)
      
    }
        */
    public void runClimberSpeed(double speed){
      climberMotor.set(speed);
    }

    @Override
    public void periodic(){
      
    }
  }

