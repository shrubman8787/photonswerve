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

    // 1 note 
    public SequentialCommandGroup Left_1n(Swerve s_Swerve, UpperSub s_Upper) {
        return new SequentialCommandGroup(
            new AutoResetEverything(s_Swerve, FieldConstants.leftSpeakerOffset),
            new AutoUpper(s_Upper, UpperState.BASE),
            new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_SHOOT_SPEED, 0.5, false)
        );
    }
    public SequentialCommandGroup Mid_1n(Swerve s_Swerve, UpperSub s_Upper) {
        return new SequentialCommandGroup(
            new AutoResetEverything(s_Swerve, FieldConstants.midSpeakerOffset),
            new AutoUpper(s_Upper, UpperState.BASE),
            new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_SHOOT_SPEED, 0.5, false)
        );
    }
    public SequentialCommandGroup Right_1n(Swerve s_Swerve, UpperSub s_Upper) {
        return new SequentialCommandGroup(
            new AutoResetEverything(s_Swerve, FieldConstants.rightSpeakerOffset),
            new AutoUpper(s_Upper, UpperState.BASE),
            new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_SHOOT_SPEED, 0.5, false)
        );
    }

    /* 2 notes */
    // blue

    public SequentialCommandGroup MidX1Base_b2n(Swerve s_Swerve, UpperSub s_Upper, VisionSub s_Vision) {
        return new SequentialCommandGroup(
            new AutoResetEverything(s_Swerve, FieldConstants.midSpeakerOffset),
            new AutoUpper(s_Upper, UpperState.BASE),
            new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_SHOOT_SPEED, FieldConstants.shootingTime, false),
            new ParallelRaceGroup(
                new SequentialCommandGroup(
                    new CustomizedDelay(0.25),
                    new AutoSwerve(s_Swerve, s_Vision, -1.35, 0, 0, false)
                ),
                new AutoUpper(s_Upper, UpperConstants.ELBOW_GROUND_POS, UpperConstants.SHOOTER_GROUND_SPEED, UpperConstants.INTAKE_GROUND_SPEED, FieldConstants.intakeTime, true)
            ),
            new ParallelRaceGroup(
                new AutoSwerve(s_Swerve, s_Vision, 0.15, 0, 0, false),
                new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_HOLD_SPEED, FieldConstants.holdTime, false)
            ),
            new AutoUpper(s_Upper, UpperState.BASE),
            new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_SHOOT_SPEED, FieldConstants.shootingTime, false)
        );
    }

    public SequentialCommandGroup MidX2Base_b2n(Swerve s_Swerve, UpperSub s_Upper, VisionSub s_Vision) { // this is where the problem occurs
        return new SequentialCommandGroup(
            new AutoResetEverything(s_Swerve, FieldConstants.midSpeakerOffset),
            new AutoUpper(s_Upper, UpperState.BASE),
            new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_SHOOT_SPEED, FieldConstants.shootingTime, false),
            new ParallelRaceGroup(
                new SequentialCommandGroup(
                    new CustomizedDelay(0.25),
                    new AutoSwerve(s_Swerve, s_Vision, -1.239, -1.179, 0.129, false) // this one
                ),
                new AutoUpper(s_Upper, UpperConstants.ELBOW_GROUND_POS, UpperConstants.SHOOTER_GROUND_SPEED, UpperConstants.INTAKE_GROUND_SPEED, FieldConstants.intakeTime, true)
            ),
            new ParallelRaceGroup(
                new AutoSwerve(s_Swerve, s_Vision, 0.15, 0, 0, false),
                new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_HOLD_SPEED, FieldConstants.holdTime, false)
            ),
            new AutoUpper(s_Upper, UpperState.BASE),
            new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_SHOOT_SPEED, FieldConstants.shootingTime, false)
        );
    }

    // red

    /* 3 notes */ 
    // blue

    public SequentialCommandGroup MidX2BaseX1Base_b3n(Swerve s_Swerve, UpperSub s_Upper, VisionSub s_Vision) {
        return new SequentialCommandGroup(
            // To X2 Then Back
            new AutoResetEverything(s_Swerve, FieldConstants.midSpeakerOffset),
            new AutoUpper(s_Upper, UpperState.BASE),
            new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_SHOOT_SPEED, FieldConstants.shootingTime, false),
            new ParallelRaceGroup(
                new SequentialCommandGroup(
                    new CustomizedDelay(0.25),
                    new AutoSwerve(s_Swerve, s_Vision, -1.4, 0, 0, false)
                ),
                new SequentialCommandGroup(
                    new CustomizedDelay(0.05),
                    new AutoUpper(s_Upper, UpperConstants.ELBOW_GROUND_POS, UpperConstants.SHOOTER_GROUND_SPEED, UpperConstants.INTAKE_GROUND_SPEED, FieldConstants.intakeTime, true)
                )
            ),
            new ParallelRaceGroup(
                new AutoSwerve(s_Swerve, s_Vision, 0.15, 0, 0, false),
                new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_HOLD_SPEED, FieldConstants.holdTime, false)
            ),
            new AutoUpper(s_Upper, UpperState.BASE),
            new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_SHOOT_SPEED, FieldConstants.shootingTime, false),
            // To X1 Then Back
            new AutoResetEverything(s_Swerve, FieldConstants.midSpeakerOffset),
            new ParallelRaceGroup(
                new SequentialCommandGroup(
                    new CustomizedDelay(0.25),
                    new AutoSwerve(s_Swerve, s_Vision, -1.239, -1.179, 0.129, false)
                ),
                new SequentialCommandGroup(
                    new CustomizedDelay(0.05),
                    new AutoUpper(s_Upper, UpperConstants.ELBOW_GROUND_POS, UpperConstants.SHOOTER_GROUND_SPEED, UpperConstants.INTAKE_GROUND_SPEED, FieldConstants.intakeTime, true)
                )
            ),
            new ParallelRaceGroup(
                new AutoSwerve(s_Swerve, s_Vision, 0, 0, 0, false),
                new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_HOLD_SPEED, FieldConstants.holdTime, false)
            ),
            new AutoUpper(s_Upper, UpperState.BASE),
            new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_SHOOT_SPEED, FieldConstants.shootingTime, false)
        );
    }
    
    // red
}