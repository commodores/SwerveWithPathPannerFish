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
        public static final double kFeederStation = 0.0;//---Position 0
        public static final double kLevel1 = 0.0;       //---Position 1
        public static final double kLevel2 = 9.0;       //---Position 2
        public static final double kLevel3 = 24.0;      //---Position 3
        public static final double kLevel4 = 27.0;      //---Position 4
        public static final double kAlgaeLow = 1.5;     //---Position 5
        public static final double kAlgaeHigh = 16.18;  //---Position 6
    }

    public static final class ArmSetpoints {
        public static final double kFeederStation = 31.00;  //---Position 0
        public static final double kLevel1 = 63;       //---Position 1
        public static final double kLevel2 = 51;      //---Position 2
        public static final double kLevel3 = 51;      //---Position 3
        public static final double kLevel4 = 212;        //---Position 4
        public static final double kAlgaeLow = 151.2609;    //---Position 5
        public static final double kAlgaeHigh = 151.2609;   //---Position 6

    }

}
