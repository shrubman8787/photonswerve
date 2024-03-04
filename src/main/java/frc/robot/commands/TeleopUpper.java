package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.ChenryLib.PID;
import frc.robot.Constants.UpperConstants;
import frc.robot.Constants.UpperState;
import frc.robot.subsystems.UpperSub;

public class TeleopUpper extends Command{

    private final UpperSub s_Upper;
    private final XboxController controller;

    private UpperState state;
    
    private double elbowAngle;
    private double intakeSpeed;
    private double shooterSpeed;

    private final PID elbowPID = new PID(
        UpperConstants.elbowKP, 
        UpperConstants.elbowKI,
        UpperConstants.elbowKD,
        UpperConstants.elbowiWindup,
        UpperConstants.elbowiLimit
    );

    public TeleopUpper(UpperSub upper, XboxController controller) {
        this.s_Upper = upper;
        this.controller = controller;
        addRequirements(upper);
    }

    @Override
    public void initialize() {
        state = UpperState.DEFAULT;
    }

    @Override
    public void execute() {

        if(controller.getYButtonPressed()) state = state == UpperState.GROUND ? UpperState.DEFAULT : UpperState.GROUND;
        if(controller.getXButtonPressed()) state = state == UpperState.AMP ? UpperState.DEFAULT : UpperState.AMP;
        if(controller.getAButtonPressed()) state = state == UpperState.SPEAKER ? UpperState.DEFAULT : UpperState.SPEAKER;
        if(controller.getRightTriggerAxis() > 0.01) state = UpperState.SHOOT;
        if(controller.getRightTriggerAxis() < 0.01 && state == UpperState.SHOOT) state = UpperState.TELE;
        if(controller.getStartButtonPressed()) state = state == UpperState.ENDGAME ? UpperState.DEFAULT : UpperState.ENDGAME;

        if(controller.getLeftBumperPressed()) state = state == UpperState.TELE ? UpperState.DEFAULT : UpperState.TELE;

        switch (state) {
            case DEFAULT:
                elbowAngle = UpperConstants.ELBOW_DEFAULT_POS;
                intakeSpeed = 0;
                shooterSpeed = 0;
                s_Upper.setLED(232, 213, 245);
                break;
            case GROUND:
                elbowAngle = UpperConstants.ELBOW_GROUND_POS;
                intakeSpeed = s_Upper.hasNote() ? 0: UpperConstants.INTAKE_GROUND_SPEED;
                shooterSpeed = UpperConstants.SHOOTER_GROUND_SPEED;
                if(s_Upper.hasNote()) s_Upper.setLED(12,41,235);
                else s_Upper.blink(12,41,235);
                if(s_Upper.hasNote()) state = UpperState.DEFAULT;
                break;
            case AMP:
                elbowAngle = UpperConstants.ELBOW_AMP_POS;
                intakeSpeed = 0;
                shooterSpeed = 0;
                s_Upper.setLED(255, 255, 0);
                break;
            case SPEAKER:
                elbowAngle = UpperConstants.ELBOW_SPEAKER_POS;
                intakeSpeed = 0;
                shooterSpeed = UpperConstants.SHOOTER_SHOOT_SPEED;
                if(Math.abs(s_Upper.getShooterRPM()) > 5000) s_Upper.setLED(255,0,0);
                else s_Upper.setLED(0,255,0);
                break;
            case SHOOT:
                intakeSpeed = UpperConstants.INTAKE_SHOOT_SPEED;
                shooterSpeed = UpperConstants.SHOOTER_SHOOT_SPEED;
                s_Upper.blink(255,0,0);
                break;
            case TELE:
                // if(controller.getLeftTriggerAxis() > 0.99) elbowAngle += 0.000976;
                // if(controller.getRightTriggerAxis() > 0.99) elbowAngle -= 0.000976;
                // if(controller.getXButton()) shooterSpeed = -1;
                // else shooterSpeed = 0;
                // if(controller.getBButton()) intakeSpeed = 1;
                // else intakeSpeed = 0;
                shooterSpeed = 0;
                intakeSpeed = 0;
                s_Upper.setLED(100, 100, 200);
                break;
            case ENDGAME:
                elbowAngle = UpperConstants.ELBOW_GROUND_POS;
                intakeSpeed = 0;
                shooterSpeed = 0;
                s_Upper.setLED(154, 0, 243);
                break;
        }

        s_Upper.setElbow(-elbowPID.calculate(elbowAngle - s_Upper.getElbowRotation()));
        s_Upper.setShooter(shooterSpeed);
        s_Upper.setIntake(intakeSpeed);

        SmartDashboard.putString("UpperState", state.toString());
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
