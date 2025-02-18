
package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import  com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.playingwithfusion.CANVenom.ControlMode;

import frc.robot.Configs;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
public class Climber extends SubsystemBase {
    private final TalonFX climberMotor = new TalonFX(13, "drivecan");
    private final Servo climberServo = new Servo(9);
    private boolean locked = false;
public Climber() {
  

  final TalonFXConfiguration climberConfig = new TalonFXConfiguration();
 
  climberConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
  climberConfig.CurrentLimits.StatorCurrentLimitEnable = true;
  climberConfig.CurrentLimits.SupplyCurrentLimit = 40;
  climberMotor.getConfigurator().apply(climberConfig);

  unLockClimber();
  


}
  

    public void climberFoward(){
      if (!locked) {
      climberMotor.set(-1);
      }
    }

    public void climberBack(){
      climberMotor.set(1);
    }

    public void stopClimber(){
      climberMotor.set(0);
    }

  
    public void lockClimber(){
      climberServo.set(1);
      locked = true;
      
    }

    public void unLockClimber(){
      climberServo.set(0.1);
      locked = false;
      
    }

    //manual methods
    public void runClimberSpeed(double speed){
      climberMotor.set(speed);
  
    }

    public void setServoPos( double pos){
      climberServo.set(pos);
      
    }



    @Override
    public void periodic(){
      
    }
  }

