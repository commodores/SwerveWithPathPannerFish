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
        public static final int climberMotor = 13;
        public static final double climberRatio = 64.0 * 3.5;  // 224:1 total
        public static final double climberDegreesPerRotation = 360.0 / climberRatio;  // 1.6071 degrees per rotation

        public static final double startAngle = 7.0;     // Initial angle from parallel
        public static final double grabAngle = 95;//64;     // Perpendicular to floor
        public static final double climbAngle = -10.0;   // 100 degrees past perpendicular
    }

    public static final class ArmivatorConstants{
        public static final int armMotor = 5;
        public static final int elevatorMotor = 3;
    }

    public static final class Level1Constants{
        public static final int level1ArmMotor = 10;
        public static final int level1IntakeMotor = 11;
    }

    public static final class ElevatorSetpoints {
        public static final double kFeederStation = 0.0;//---Position 0
        public static final double kLevel1 = 0.0;       //---Position 1
        public static final double kLevel2 = 10.0;       //---Position 2
        public static final double kLevel3 = 27.0;      //---Position 3
        public static final double kLevel4 = 27.0;      //---Position 4
        public static final double kAlgaeLow = 1.5;     //---Position 5
        public static final double kAlgaeHigh = 16.18;  //---Position 6
    }

    public static final class ArmSetpoints {
        public static final double kFeederStation = 27.00;  //---Position 0
        public static final double kLevel1 = 63;       //---Position 1
        public static final double kLevel2 = 51;      //---Position 2
        public static final double kLevel3 = 51;      //---Position 3
        public static final double kLevel4 = 214;        //---Position 4
        public static final double kAlgaeLow = 165;    //---Position 5
        public static final double kAlgaeHigh = 165;   //---Position 6

    }

    public static final class Level1ArmSetpoints {
        public static final double kStow = 125;  //---Position 0
        public static final double kFloor = 7;       //---Position 1
        public static final double kScore = 108;      //---Position 2        
    }

    public static final class AlignToBranchConstants {
        public static final double X_REEF_ALIGNMENT_P = 1.5;//.5;
        public static final double Y_REEF_ALIGNMENT_P = 3;//1;
        public static final double ROT_REEF_ALIGNMENT_P = 0.1625;

        public static final double ROT_SETPOINT_REEF_ALIGNMENT = 1.5;  // Rotation
        public static final double ROT_TOLERANCE_REEF_ALIGNMENT = 1;
        public static final double X_SETPOINT_REEF_ALIGNMENT = 0.19;//-0.34;  // Vertical pose
        public static final double X_TOLERANCE_REEF_ALIGNMENT = 0.03;//0.02;
        public static final double Y_SETPOINT_REEF_ALIGNMENT = -0.52;//0.16;  // Horizontal pose
        public static final double Y_TOLERANCE_REEF_ALIGNMENT =0.03;// 0.02;


        public static final double DONT_SEE_TAG_WAIT_TIME = 1;
        public static final double POSE_VALIDATION_TIME = 2;//0.3;
    }

}
