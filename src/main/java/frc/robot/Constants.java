// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class Constants {

    public static final class IntakeConstants{
        public static final int intakeMotor = 6;
        public static final int hopperMotor = 9;
    }

    public static final class ClimberConstants{
        public static final int climberMotor = 2;
    }

    public static final class ArmivatorConstants{
        public static final int armMotor = 5;
        public static final int elevatorMotor = 3;
    }

    public static final class ElevatorSetpoints {
        public static final double kFeederStation = 0;
        public static final double kNeutralPosition = 0;
        public static final double kLevel1 = 0;
        public static final double kLevel2 = 0.257;
        public static final double kLevel3 = 0.646;
        public static final double kLevel4 = 0.71;
    }

    public static final class ArmSetpoints {
        public static final double kFeederStation = 1;///.436;
        public static final double kNeutralPosition = 1;//.846;
        public static final double kLevel1 = 1;
        public static final double kLevel2 = 1;
        public static final double kLevel3 = 1;
        public static final double kLevel4 = 1;//3.31;
    }

}
