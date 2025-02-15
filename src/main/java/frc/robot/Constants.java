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
        public static final int kFeederStation = 0;
        public static final int kNeutralPosition = 0;
        public static final int kLevel1 = 0;
        public static final int kLevel2 = 59;
        public static final int kLevel3 = 121;
        public static final int kLevel4 = 132;
    }

    public static final class ArmSetpoints {
        public static final double kFeederStation = .436;
        public static final double kNeutralPosition = .846;
        public static final double kLevel1 = .846;
        public static final double kLevel2 = .846;
        public static final double kLevel3 = 0.846;
        public static final double kLevel4 = 3.31;
    }

}
