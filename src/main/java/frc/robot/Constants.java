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
        public static final double kLevel1 = 0;
        public static final double kLevel2 = 9;//26;
        public static final double kLevel3 = 20; //60;
        public static final double kLevel4 = 20; //60;
        public static final double kAlgaeLow = 0;
        public static final double kAlgaeHigh = 11; //34.3;
    }

    public static final class ArmSetpoints {
        public static final double kFeederStation = .441;
        public static final double kLevel1 = 1.2;
        public static final double kLevel2 = .846;
        public static final double kLevel3 = .846;
        public static final double kLevel4 = 3.58;
        public static final double kAlgaeLow = 2.64;
        public static final double kAlgaeHigh = 2.64;

    }

}
