package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.UpperConstants;
import frc.robot.Constants.UpperState;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.UpperSub;

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
    public SequentialCommandGroup MidX1Base_2n(Swerve s_Swerve, UpperSub s_Upper) { // this is where the problem occurs
        return new SequentialCommandGroup(
            new AutoResetEverything(s_Swerve, FieldConstants.midSpeakerOffset),
            new AutoUpper(s_Upper, UpperState.BASE),
            new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_SHOOT_SPEED, FieldConstants.shootingTime, false),
            new ParallelRaceGroup(
                new AutoSwerve(s_Swerve, -1.5577, -1.539, 0, 1.25, 0),
                new AutoUpper(s_Upper, UpperConstants.ELBOW_GROUND_POS, UpperConstants.SHOOTER_GROUND_SPEED, UpperConstants.INTAKE_GROUND_SPEED, FieldConstants.intakeTime, true)
            ),
            new ParallelRaceGroup(
                new AutoSwerve(s_Swerve, 0.15, -0.2, 0),
                new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_HOLD_SPEED, FieldConstants.holdTime, false)
            ),
            new AutoUpper(s_Upper, UpperState.BASE),
            new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_SHOOT_SPEED, FieldConstants.shootingTime, false),
            new AutoResetEverything(s_Swerve, FieldConstants.midSpeakerOffset)
        );
    }

    public SequentialCommandGroup MidX2Base_2n(Swerve s_Swerve, UpperSub s_Upper) {
        return new SequentialCommandGroup(
            new AutoResetEverything(s_Swerve, FieldConstants.midSpeakerOffset),
            new AutoUpper(s_Upper, UpperState.BASE),
            new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_SHOOT_SPEED, FieldConstants.shootingTime, false),
            new ParallelRaceGroup(
                new SequentialCommandGroup(
                    new CustomizedDelay(0.25),
                    new AutoSwerve(s_Swerve, -1.55, 0, 0)
                ),
                new AutoUpper(s_Upper, UpperConstants.ELBOW_GROUND_POS, UpperConstants.SHOOTER_GROUND_SPEED, UpperConstants.INTAKE_GROUND_SPEED, FieldConstants.intakeTime, true)
            ),
            new ParallelRaceGroup(
                new AutoSwerve(s_Swerve, 0.15, 0, 0),
                new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_HOLD_SPEED, FieldConstants.holdTime, false)
            ),
            new AutoUpper(s_Upper, UpperState.BASE),
            new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_SHOOT_SPEED, FieldConstants.shootingTime, false),
            new AutoResetEverything(s_Swerve, FieldConstants.midSpeakerOffset)
        );
    }

    public SequentialCommandGroup MidX3Base_2n(Swerve s_Swerve, UpperSub s_Upper) {
        return new SequentialCommandGroup(
            new AutoResetEverything(s_Swerve, FieldConstants.midSpeakerOffset),
            new AutoUpper(s_Upper, UpperState.BASE),
            new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_SHOOT_SPEED, FieldConstants.shootingTime, false),
            new ParallelRaceGroup(
                new AutoSwerve(s_Swerve, -1.5577, 1.639, 0, 1, 0),
                new AutoUpper(s_Upper, UpperConstants.ELBOW_GROUND_POS, UpperConstants.SHOOTER_GROUND_SPEED, UpperConstants.INTAKE_GROUND_SPEED, FieldConstants.intakeTime, true)
            ),
            new ParallelRaceGroup(
                new AutoSwerve(s_Swerve, 0.175, 0, 0),
                new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_HOLD_SPEED, FieldConstants.holdTime, false)
            ),
            new AutoUpper(s_Upper, UpperState.BASE),
            new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_SHOOT_SPEED, FieldConstants.shootingTime, false),
            new AutoResetEverything(s_Swerve, FieldConstants.midSpeakerOffset)
        );
    }

    /* 3 notes */
    public SequentialCommandGroup MidX2BaseX1Base_3n(Swerve s_Swerve, UpperSub s_Upper) {
        return new SequentialCommandGroup(
            // To X2 Then Back
            new AutoResetEverything(s_Swerve, FieldConstants.midSpeakerOffset),
            new AutoUpper(s_Upper, UpperState.BASE),
            new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_SHOOT_SPEED, FieldConstants.shootingTime, false),
            new ParallelRaceGroup(
                new SequentialCommandGroup(
                    new CustomizedDelay(0.35),
                    new AutoSwerve(s_Swerve, -1.55, 0, 0)
                ),
                new AutoUpper(s_Upper, UpperConstants.ELBOW_GROUND_POS, UpperConstants.SHOOTER_GROUND_SPEED, UpperConstants.INTAKE_GROUND_SPEED, FieldConstants.intakeTime, true)
            ),
            new ParallelRaceGroup(
                new AutoSwerve(s_Swerve, 0.3, 0.05, 0),
                new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_HOLD_SPEED, FieldConstants.holdTime, false)
            ),
            new AutoUpper(s_Upper, UpperState.BASE),
            new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_SHOOT_SPEED, FieldConstants.shootingTime, false),
            // To X1 Then Back
            new AutoResetEverything(s_Swerve, FieldConstants.midSpeakerOffset),
            new ParallelRaceGroup(
                new AutoSwerve(s_Swerve, -1.525, -1.539, 0, 1, 0),
                new AutoUpper(s_Upper, UpperConstants.ELBOW_GROUND_POS, UpperConstants.SHOOTER_GROUND_SPEED, UpperConstants.INTAKE_GROUND_SPEED, FieldConstants.intakeTime, true)
            ),
            new ParallelRaceGroup(
                new AutoSwerve(s_Swerve, 0.15, 0.125, 0),
                new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_HOLD_SPEED, FieldConstants.holdTime, false)
            ),
            new AutoUpper(s_Upper, UpperState.BASE),
            new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_SHOOT_SPEED, FieldConstants.shootingTime, false),
            new AutoUpper(s_Upper, UpperConstants.ELBOW_DEFAULT_POS, 0, 0, 2, false),
            new AutoResetEverything(s_Swerve, FieldConstants.midSpeakerOffset)
        );
    }

    public SequentialCommandGroup MidX2BaseX3Base_3n(Swerve s_Swerve, UpperSub s_Upper) {
        return new SequentialCommandGroup(
            // To X2 Then Back
            new AutoResetEverything(s_Swerve, FieldConstants.midSpeakerOffset),
            new AutoUpper(s_Upper, UpperState.BASE),
            new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_SHOOT_SPEED, FieldConstants.shootingTime, false),
            new ParallelRaceGroup(
                new SequentialCommandGroup(
                    new CustomizedDelay(0.35),
                    new AutoSwerve(s_Swerve, -1.55, 0, 0)
                ),
                new AutoUpper(s_Upper, UpperConstants.ELBOW_GROUND_POS, UpperConstants.SHOOTER_GROUND_SPEED, UpperConstants.INTAKE_GROUND_SPEED, FieldConstants.intakeTime, true)
            ),
            new ParallelRaceGroup(
                new AutoSwerve(s_Swerve, 0.3, 0.05, 0),
                new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_HOLD_SPEED, FieldConstants.holdTime, false)
            ),
            new AutoUpper(s_Upper, UpperState.BASE),
            new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_SHOOT_SPEED, FieldConstants.shootingTime, false),
            // To X3 Then Back
            new AutoResetEverything(s_Swerve, FieldConstants.midSpeakerOffset),
            new ParallelRaceGroup(
                new AutoSwerve(s_Swerve, -1.575, 1.619, 0, 1, 0),
                new AutoUpper(s_Upper, UpperConstants.ELBOW_GROUND_POS, UpperConstants.SHOOTER_GROUND_SPEED, UpperConstants.INTAKE_GROUND_SPEED, FieldConstants.intakeTime, true)
            ),
            new ParallelRaceGroup(
                new AutoSwerve(s_Swerve, 0.15, 0.125, 0),
                new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_HOLD_SPEED, FieldConstants.holdTime, false)
            ),
            new AutoUpper(s_Upper, UpperState.BASE),
            new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_SHOOT_SPEED, FieldConstants.shootingTime, false),
            new AutoUpper(s_Upper, UpperConstants.ELBOW_DEFAULT_POS, 0, 0, 2, false),
            new AutoResetEverything(s_Swerve, FieldConstants.midSpeakerOffset)
        );
    }

    /* 4 note */
    public SequentialCommandGroup MidX2BaseX1BaseX3Base_4n(Swerve s_Swerve, UpperSub s_Upper) {
        return new SequentialCommandGroup(
            // To X2 Then Back
            new AutoResetEverything(s_Swerve, FieldConstants.midSpeakerOffset),
            new AutoUpper(s_Upper, UpperState.BASE),
            new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_SHOOT_SPEED, FieldConstants.shootingTime, false),
            new ParallelRaceGroup(
                new SequentialCommandGroup(
                    new CustomizedDelay(0.35),
                    new AutoSwerve(s_Swerve, -1.55, 0, 0)
                ),
                new AutoUpper(s_Upper, UpperConstants.ELBOW_GROUND_POS, UpperConstants.SHOOTER_GROUND_SPEED, UpperConstants.INTAKE_GROUND_SPEED, FieldConstants.intakeTime, true)
            ),
            new ParallelRaceGroup(
                new AutoSwerve(s_Swerve, 0.3, 0.05, 0),
                new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_HOLD_SPEED, FieldConstants.holdTime, false)
            ),
            new AutoUpper(s_Upper, UpperState.BASE),
            new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_SHOOT_SPEED, FieldConstants.shootingTime, false),
            // To X1 Then Back
            new AutoResetEverything(s_Swerve, FieldConstants.midSpeakerOffset),
            new ParallelRaceGroup(
                new AutoSwerve(s_Swerve, -1.595, -1.539, 0, 1, 0),
                new AutoUpper(s_Upper, UpperConstants.ELBOW_GROUND_POS, UpperConstants.SHOOTER_GROUND_SPEED, UpperConstants.INTAKE_GROUND_SPEED, FieldConstants.intakeTime, true)
            ),
            new ParallelRaceGroup(
                new AutoSwerve(s_Swerve, 0.15, 0.125, 0),
                new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_HOLD_SPEED, FieldConstants.holdTime, false)
            ),
            new AutoUpper(s_Upper, UpperState.BASE),
            new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_SHOOT_SPEED, FieldConstants.shootingTime, false),
            // To X3 Then Back
            new AutoResetEverything(s_Swerve, FieldConstants.midSpeakerOffset),
            new ParallelRaceGroup(
                new AutoSwerve(s_Swerve, -1.575, 1.619, 0, 1, 0),
                new AutoUpper(s_Upper, UpperConstants.ELBOW_GROUND_POS, UpperConstants.SHOOTER_GROUND_SPEED, UpperConstants.INTAKE_GROUND_SPEED, FieldConstants.intakeTime, true)
            ),
            new ParallelRaceGroup(
                new AutoSwerve(s_Swerve, 0.15, 0, 0),
                new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_HOLD_SPEED, FieldConstants.holdTime, false)
            ),
            new AutoUpper(s_Upper, UpperState.BASE),
            new AutoUpper(s_Upper, UpperConstants.ELBOW_BASE_POS, UpperConstants.SHOOTER_SHOOT_SPEED, UpperConstants.INTAKE_SHOOT_SPEED, FieldConstants.shootingTime, false),
            new AutoUpper(s_Upper, UpperConstants.ELBOW_DEFAULT_POS, 0, 0, 2, false),
            new AutoResetEverything(s_Swerve, FieldConstants.midSpeakerOffset)
        );
    }
}