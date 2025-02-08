
package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import  com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
public class Climber extends SubsystemBase {
    private final  SparkFlex climberMotor = new SparkFlex(Constants.ClimberConstants.climberMotor, MotorType.kBrushless);

public Climber() {


    configureSparkFlex(climberMotor);
        
    }
    
    
    private void configureSparkFlex(SparkFlex climberMotor) {
        SparkFlexConfig config = new SparkFlexConfig();
        config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(100);
        climberMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        

      
    }
    public void runClimberSpeed(double speed){
      climberMotor.set(speed);
    }

    @Override
    public void periodic(){
      
    }
  }

