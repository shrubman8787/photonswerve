package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.UpperConstants;
import frc.robot.Constants.UpperState;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.UpperSub;
import frc.robot.subsystems.VisionSub;

public class Autos {

    /* 1 note */
    public SequentialCommandGroup Left_1n(Swerve s_Swerve, UpperSub s_Upper) {
        return new SequentialCommandGroup(
            new AutoResetEverything(s_Swerve, FieldConstants.leftSpeakerOffset),
            new AutoUpper(s_Upper, UpperState.BASE),
            new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_SHOOT_SPEED, 0.5)
        );
    }
    public SequentialCommandGroup Mid_1n(Swerve s_Swerve, UpperSub s_Upper) {
        return new SequentialCommandGroup(
            new AutoResetEverything(s_Swerve, FieldConstants.midSpeakerOffset),
            new AutoUpper(s_Upper, UpperState.BASE),
            new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_SHOOT_SPEED, 0.5)
        );
    }
    public SequentialCommandGroup Right_1n(Swerve s_Swerve, UpperSub s_Upper) {
        return new SequentialCommandGroup(
            new AutoResetEverything(s_Swerve, FieldConstants.rightSpeakerOffset),
            new AutoUpper(s_Upper, UpperState.BASE),
            new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_SHOOT_SPEED, 0.5)
        );
    }

    /* 2 notes */
    // blue
    public SequentialCommandGroup LeftX1Base_b2n(Swerve s_Swerve, UpperSub s_Upper, VisionSub s_Vision) {
        return new SequentialCommandGroup(
            new AutoResetEverything(s_Swerve, FieldConstants.leftSpeakerOffset),
            new AutoUpper(s_Upper, UpperState.BASE),
            new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_SHOOT_SPEED, FieldConstants.shootingTime),
            new ParallelRaceGroup(
                new SequentialCommandGroup(
                    new CustomizedDelay(0.5),
                    new AutoSwerve(s_Swerve, s_Vision, FieldConstants.BX1X, FieldConstants.BX1Y, FieldConstants.BX1Z, false)
                ),
                new AutoUpper(s_Upper, UpperConstants.ELBOW_GROUND_POS, UpperConstants.SHOOTER_GROUND_SPEED, UpperConstants.INTAKE_GROUND_SPEED, FieldConstants.intakeTime)
            ),
            new ParallelCommandGroup(
                new AutoSwerve(s_Swerve, s_Vision, -FieldConstants.BX1X*0.4, -FieldConstants.BX1Y*0.4, -FieldConstants.BX1Z*0.4, false),
                new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_HOLD_SPEED, FieldConstants.holdTime)
            ),
            new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_SHOOT_SPEED, FieldConstants.shootingTime)
        );
    }

    public SequentialCommandGroup MidX2Base_b2n(Swerve s_Swerve, UpperSub s_Upper, VisionSub s_Vision) {
        return new SequentialCommandGroup(
            new AutoResetEverything(s_Swerve, FieldConstants.midSpeakerOffset),
            new AutoUpper(s_Upper, UpperState.BASE),
            new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_SHOOT_SPEED, FieldConstants.shootingTime),
            new ParallelRaceGroup(
                new SequentialCommandGroup(
                    new CustomizedDelay(0.5),
                    new AutoSwerve(s_Swerve, s_Vision, -1.3, FieldConstants.BX2Y, FieldConstants.BX2Z, false)
                ),
                new AutoUpper(s_Upper, UpperConstants.ELBOW_GROUND_POS, UpperConstants.SHOOTER_GROUND_SPEED, UpperConstants.INTAKE_GROUND_SPEED, FieldConstants.intakeTime)
            ),
            new ParallelRaceGroup(
                new AutoSwerve(s_Swerve, s_Vision, 1, -FieldConstants.BX2Y, -FieldConstants.BX2Z, false),
                new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_HOLD_SPEED, FieldConstants.holdTime)
            ),
            new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_SHOOT_SPEED, FieldConstants.shootingTime)
        );
    }

    public SequentialCommandGroup RightX3Base_b2n(Swerve s_Swerve, UpperSub s_Upper, VisionSub s_Vision) {
        return new SequentialCommandGroup(
            new AutoResetEverything(s_Swerve, FieldConstants.rightSpeakerOffset),
            new AutoUpper(s_Upper, UpperState.BASE),
            new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_SHOOT_SPEED, FieldConstants.shootingTime),
            new ParallelRaceGroup(
                new SequentialCommandGroup(
                    new CustomizedDelay(0.5),
                    new AutoSwerve(s_Swerve, s_Vision, FieldConstants.BX3X, FieldConstants.BX3Y, FieldConstants.BX3Z, false)
                ),
                new AutoUpper(s_Upper, UpperConstants.ELBOW_GROUND_POS, UpperConstants.SHOOTER_GROUND_SPEED, UpperConstants.INTAKE_GROUND_SPEED, FieldConstants.intakeTime)
            ),
            new ParallelCommandGroup(
                new AutoSwerve(s_Swerve, s_Vision, -FieldConstants.BX3X*0.4, -FieldConstants.BX3Y*0.4, -FieldConstants.BX3Z*0.4, false),
                new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_HOLD_SPEED, FieldConstants.holdTime)
            ),
            new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_SHOOT_SPEED, FieldConstants.shootingTime)
        );
    }

    // red

    /* 3 notes */
    // blue
    public SequentialCommandGroup LeftX1Y1Base_b3n(Swerve s_Swerve, UpperSub s_Upper, VisionSub s_Vision) {
        return new SequentialCommandGroup(
            new AutoResetEverything(s_Swerve, FieldConstants.leftSpeakerOffset),
            new AutoUpper(s_Upper, UpperState.BASE),
            new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_SHOOT_SPEED, 0.5),
            new ParallelRaceGroup(
                new AutoSwerve(s_Swerve, s_Vision, FieldConstants.BX1X, FieldConstants.BX1Y, FieldConstants.BX1Z, false), // need to change
                new AutoUpper(s_Upper, UpperConstants.ELBOW_GROUND_POS, UpperConstants.SHOOTER_GROUND_SPEED, UpperConstants.INTAKE_GROUND_SPEED, 5)
            ),
            new ParallelRaceGroup(
                new AutoSwerve(s_Swerve, s_Vision, FieldConstants.leftSpeakerX, FieldConstants.leftSpeakerY, FieldConstants.leftSpeakerZ, false),
                new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_HOLD_SPEED, FieldConstants.holdTime)
            ),
            new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_SHOOT_SPEED, 0.5),
            new ParallelRaceGroup(
                new AutoSwerve(s_Swerve, s_Vision, FieldConstants.BY1X, FieldConstants.BY1Y, FieldConstants.BY1Z, false),
                new AutoUpper(s_Upper, UpperConstants.ELBOW_GROUND_POS, UpperConstants.SHOOTER_GROUND_SPEED, UpperConstants.INTAKE_GROUND_SPEED, 5)
            ),
            new ParallelRaceGroup(
                new AutoSwerve(s_Swerve, s_Vision, FieldConstants.leftSpeakerX, FieldConstants.leftSpeakerY, FieldConstants.leftSpeakerZ, false),
                new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_HOLD_SPEED, FieldConstants.holdTime)
            ),
            new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_SHOOT_SPEED, 0.5)
        );
    }

    // these commands are all fucked i think
    // red
    
    // public SequentialCommandGroup X1Y1_r(Swerve s_Swerve, UpperSub s_Upper, VisionSub s_Vision) {
    //     return new SequentialCommandGroup(
    //         new AutoResetEverything(s_Swerve),
    //         new AutoUpper(s_Upper, UpperState.BASE),
    //         new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, -1, 1, 0.5),
    //         new ParallelRaceGroup(
    //             new AutoSwerve(s_Swerve, s_Vision, -1.05, 1.18, -0.2, false),
    //             new AutoUpper(s_Upper, UpperConstants.ELBOW_GROUND_POS, 0.03, 0.3, 5)
    //         ),                                
    //         new ParallelCommandGroup(
    //             new AutoSwerve(s_Swerve, s_Vision, -1.03, 1.12, -0.07, false),
    //             new AutoUpper(s_Upper, -0.204589, 0, 0, 1.5)
    //         ),
    //         new AutoUpper(s_Upper, -0.16, -1, 0, 1.5),
    //         new AutoUpper(s_Upper, -0.16, -1, 1, 0.5)
    //     );
    // }

    // public SequentialCommandGroup X2Y2Z3_r(Swerve s_Swerve, UpperSub s_Upper, VisionSub s_Vision) {
    //     return new SequentialCommandGroup(
    //         new AutoResetEverything(s_Swerve),
    //         new AutoUpper(s_Upper, UpperState.BASE),
    //         new AutoUpper(s_Upper, -0.204589, -1, 1, 0.5),
    //         new ParallelRaceGroup(
    //             new AutoSwerve(s_Swerve, s_Vision, -0.92, 0, 0, false),
    //             new AutoUpper(s_Upper, -0.236328, 0.03, 0.3, 5)
    //         ),
    //         new AutoUpper(s_Upper, -0.16, -1, 0, 1.5),
    //         new AutoUpper(s_Upper, -0.16, -1, 1, 0.5),
    //         new ParallelRaceGroup(
    //             new AutoSwerve(s_Swerve, s_Vision, -6.68, 1.57, 0, false),
    //             new AutoUpper(s_Upper, -0.236328, 0.03, 0.3, 5)
    //         ),
    //         new ParallelRaceGroup(
    //             new AutoSwerve(s_Swerve, s_Vision, -4, 1.57, 0, false),
    //             new AutoUpper(s_Upper, -0.204589, -1, 0, 5)
    //         ),
    //         new ParallelRaceGroup(
    //             new AutoSwerve(s_Swerve, s_Vision, 0, 0, 0, false),
    //             new AutoUpper(s_Upper, -0.204589, -1, 0, 5)
    //         ),
    //         new AutoUpper(s_Upper, -0.204589, -1, 1, 0.5)
    //     );
    // }

    // public SequentialCommandGroup X3Y3_r(Swerve s_Swerve, UpperSub s_Upper, VisionSub s_Vision) {
    //     return new SequentialCommandGroup(
    //         new AutoResetEverything(s_Swerve),
    //         new AutoUpper(s_Upper, UpperState.BASE),
    //         new AutoUpper(s_Upper, -0.204589, -1, 1, 0.5),
    //         new ParallelRaceGroup(
    //             new AutoSwerve(s_Swerve, s_Vision, -1.07, -1.20, 0.16, false),
    //             new AutoUpper(s_Upper, -0.236328, 0.03, 0.3, 5)
    //             ),
    //             new ParallelCommandGroup(
    //                 new AutoSwerve(s_Swerve, s_Vision, -1.03, -1.12, 0.07, false),
    //                 new AutoUpper(s_Upper, -0.204589, 0, 0, 1.5)
    //                 ),
    //                 new AutoUpper(s_Upper, -0.16, -1, 0, 1.5),
    //                 new AutoUpper(s_Upper, -0.16, -1, 1, 0.5)
    //                 );
    // }

    // public SequentialCommandGroup X3Leave_r(Swerve s_Swerve, UpperSub s_Upper, VisionSub s_Vision) {
    //     return new SequentialCommandGroup(
    //         new AutoResetEverything(s_Swerve),
    //         new AutoUpper(s_Upper, -0.204589, -1, 1, 0.5),
    //         new AutoSwerve(s_Swerve, s_Vision, 0, -2, 0, false),
    //         new AutoSwerve(s_Swerve, s_Vision, -3, 0, 0, false)
    //     );
    // }
                
    // // blue

    // public SequentialCommandGroup X1Y1_b(Swerve s_Swerve, UpperSub s_Upper, VisionSub s_Vision) {
    //     return new SequentialCommandGroup(
    //         new AutoResetEverything(s_Swerve),
    //         new AutoUpper(s_Upper, UpperState.BASE),
    //         new AutoUpper(s_Upper, -0.204589, -1, 1, 0.5),
    //         new ParallelRaceGroup(
    //             new AutoSwerve(s_Swerve, s_Vision, -1.07, 1.20, -0.16, false),
    //             new AutoUpper(s_Upper, -0.236328, 0.03, 0.3, 5)
    //         ),
    //         new ParallelCommandGroup(
    //             new AutoSwerve(s_Swerve, s_Vision, -1.03, 1.12, -0.07, false),
    //             new AutoUpper(s_Upper, -0.204589, 0, 0, 1.5)
    //         ),
    //         new AutoUpper(s_Upper, -0.16, -1, 0, 1.5),
    //         new AutoUpper(s_Upper, -0.16, -1, 1, 0.5)
    //     );
    // }

    // public SequentialCommandGroup X2Y2Z3_b(Swerve s_Swerve, UpperSub s_Upper, VisionSub s_Vision) {
    //     return new SequentialCommandGroup(
    //         new AutoResetEverything(s_Swerve),
    //         new AutoUpper(s_Upper, UpperState.BASE),
    //         new AutoUpper(s_Upper, -0.204589, -1, 1, 0.5),
    //         new ParallelRaceGroup(
    //             new AutoSwerve(s_Swerve, s_Vision, -0.92, 0, 0, false),
    //             new AutoUpper(s_Upper, -0.236328, 0.03, 0.3, 5)
    //         ),
    //         new AutoUpper(s_Upper, -0.16, -1, 0, 1.5),
    //         new AutoUpper(s_Upper, -0.16, -1, 1, 0.5),
    //         new ParallelRaceGroup(
    //             new AutoSwerve(s_Swerve, s_Vision, -6.68, -1.57, 0, false),
    //             new AutoUpper(s_Upper, -0.236328, 0.03, 0.3, 5)
    //         ),
    //         new ParallelRaceGroup(
    //             new AutoSwerve(s_Swerve, s_Vision, -4, -1.57, 0, false),
    //             new AutoUpper(s_Upper, -0.204589, -1, 0, 5)
    //         ),
    //         new ParallelRaceGroup(
    //             new AutoSwerve(s_Swerve, s_Vision, 0, 0, 0, false),
    //             new AutoUpper(s_Upper, -0.204589, -1, 0, 5)
    //         ),
    //         new AutoUpper(s_Upper, -0.204589, -1, 1, 0.5)
    //     );
    // }

    // public SequentialCommandGroup X3Y3_b(Swerve s_Swerve, UpperSub s_Upper, VisionSub s_Vision) {
    //     return new SequentialCommandGroup(
    //         new AutoResetEverything(s_Swerve),
    //         new AutoUpper(s_Upper, UpperState.BASE),
    //         new AutoUpper(s_Upper, -0.204589, -1, 1, 0.5),
    //         new ParallelRaceGroup(
    //             new AutoSwerve(s_Swerve, s_Vision, -1.05, -1.18, 0.2, false),
    //             new AutoUpper(s_Upper, -0.236328, 0.03, 0.3, 5)
    //         ),
    //         new ParallelCommandGroup(
    //             new AutoSwerve(s_Swerve, s_Vision, -1.03, -1.12, 0.07, false),
    //             new AutoUpper(s_Upper, -0.204589, 0, 0, 1.5)
    //         ),
    //         new AutoUpper(s_Upper, -0.16, -1, 0, 1.5),
    //         new AutoUpper(s_Upper, -0.16, -1, 1, 0.5)
    //     );
    // }

    // public SequentialCommandGroup X3Leave_b(Swerve s_Swerve, UpperSub s_Upper, VisionSub s_Vision) {
    //     return new SequentialCommandGroup(
    //         new AutoResetEverything(s_Swerve),
    //         new AutoUpper(s_Upper, UpperState.BASE),
    //         new AutoUpper(s_Upper, -0.204589, -1, 1, 0.5),
    //         new AutoSwerve(s_Swerve, s_Vision, -1.732, 1, 0, false),
    //         new AutoSwerve(s_Swerve, s_Vision, -1.5, -2.598, 0, false)
    //     );
    // }     
}

    // public SequentialCommandGroup X2Y1Y2Y3_r(Swerve s_Swerve, UpperSub s_Upper, VisionSub s_Vision1) {
    //     return new SequentialCommandGroup(
    //         new AutoUpper(s_Upper, UpperState.SPEAKER),
    //         new AutoUpper(s_Upper, -0.204589, -1, 1, 0.5),
    //         new ParallelRaceGroup(
    //             new AutoSwerve(s_Swerve, s_Vision, -1, 1, 0.132),
    //             new AutoUpper(s_Upper, -0.236328, 0.03, 0.3, 5)
    //         ),
    //         new ParallelRaceGroup(
    //             new AutoSwerve(s_Swerve, s_Vision, 0, 0, 0),
    //             new AutoUpper(s_Upper, -0.204589, -1, 0, 5)
    //         )
    //     ); 
    // }