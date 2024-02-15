// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class Operator {
        public static final int RightFlightStickControllerPort = 1;
        public static final int LeftFlightStickControllerPort = 2;
        public static final int OperatorControllerPort = 0;
    }

    public static class CANIDS {
        public static final int TopIntakeMotor = 2;
        public static final int BottomIntakeMotor = 1;
        public static final int TopShooterMotor = 5;
        public static final int BottomShooterMotor = 4;
        public static final int IndexerMotor = 3;
    }

    public static class Sensors {
        public static final int RightDriveEncoder = 0;
        public static final int LeftDriveEncoder = 0;
        public static final int TopShooterEncoder = 0;
        public static final int BottomShooterEncoder = 0;
    }

    public static class DriveTrain {
        //left
        public static final int LeftMotor1 = 9;
        public static final int LeftMotor2 = 8;
        public static final int LeftRelativeEncoder = 0;
        //right
        public static final int RightMotor1 = 7;
        public static final int RightMotor2 = 6;
        public static final int RightRelativeEncoder = 0;
    }

    public static class Intake {
        public static final double IntakeOffsetMultiplicative = 0.95;
        public static final double TriggerDeadZone = 0.2;
    }
    public static class Shooter {
         public static final double LowSpeed = -.2;
         public static final double HighSpeed = -1;
    }
    public static class Indexer {
        public static final double IndexerSpeed = 0.8;
    }
    



}
