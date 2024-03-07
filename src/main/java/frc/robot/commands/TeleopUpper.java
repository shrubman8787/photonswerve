package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.ChenryLib.PID;
import frc.robot.Constants;
import frc.robot.Constants.UpperConstants;
import frc.robot.Constants.UpperState;
import frc.robot.subsystems.UpperSub;
import frc.robot.subsystems.VisionSub;

public class TeleopUpper extends Command{

    private final UpperSub s_Upper;
    private final VisionSub s_Vision;
    private final XboxController controller;
    
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

    public TeleopUpper(UpperSub upper, VisionSub vision, XboxController controller) {
        this.s_Upper = upper;
        this.s_Vision = vision;
        this.controller = controller;
        addRequirements(upper);
    }

    @Override
    public void initialize() {
        Constants.state = UpperState.DEFAULT;
    }

    @Override
    public void execute() {

        if(Constants.state != UpperState.TELE) {
            if(controller.getYButtonPressed()) Constants.state = Constants.state == UpperState.GROUND ? UpperState.DEFAULT : UpperState.GROUND;
            if(controller.getXButtonPressed()) Constants.state = Constants.state == UpperState.AMP ? UpperState.DEFAULT : UpperState.AMP;
            if(controller.getAButtonPressed()) Constants.state = Constants.state == UpperState.BASE ? UpperState.DEFAULT : UpperState.BASE;
            if(controller.getBButtonPressed()) Constants.state = Constants.state == UpperState.AUTO ? UpperState.DEFAULT : UpperState.AUTO;
            // if(controller.getLeftBumperPressed()) Constants.state = Constants.state == UpperState.TRANSPORT ? UpperState.DEFAULT : UpperState.TRANSPORT;
            if(controller.getRightTriggerAxis() > 0.01) Constants.state = UpperState.SHOOT;
            if(controller.getRightTriggerAxis() < 0.01 && Constants.state == UpperState.SHOOT) Constants.state = UpperState.TELE;
            if(controller.getStartButtonPressed()) Constants.state = Constants.state == UpperState.ENDGAME ? UpperState.DEFAULT : UpperState.ENDGAME;
        }

        if(controller.getLeftBumperPressed()) Constants.state = Constants.state == UpperState.TELE ? UpperState.DEFAULT : UpperState.TELE;

        switch (Constants.state) {
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
                if(s_Upper.hasNote()) Constants.state = UpperState.DEFAULT;
                break;
            case AMP:
                elbowAngle = UpperConstants.ELBOW_AMP_POS;
                intakeSpeed = 0;
                shooterSpeed = 0;
                s_Upper.setLED(255, 255, 0);
                break;
            case BASE:
                elbowAngle = UpperConstants.ELBOW_BASE_POS;
                intakeSpeed = 0;
                shooterSpeed = UpperConstants.SHOOTER_SHOOT_SPEED;
                if(Math.abs(s_Upper.getShooterRPM()) > UpperConstants.SHOOTER_LEGAL_SPEED) s_Upper.setLED(0,255,0);
                else s_Upper.setLED(255,0,0);
                break;
            case TRANSPORT:
                elbowAngle = UpperConstants.ELBOW_TRANSPORT_POS;
                intakeSpeed = 0;
                shooterSpeed = UpperConstants.SHOOTER_SHOOT_SPEED;
                if(Math.abs(s_Upper.getShooterRPM()) > UpperConstants.SHOOTER_LEGAL_SPEED) s_Upper.setLED(0, 255, 0);
                else s_Upper.setLED(0, 0, 0);
                break;
            case AUTO:
                elbowAngle = s_Vision.calculateAutoAiming();
                intakeSpeed = 0;
                shooterSpeed = UpperConstants.SHOOTER_SHOOT_SPEED;
                if(Math.abs(s_Upper.getShooterRPM()) > UpperConstants.SHOOTER_LEGAL_SPEED) s_Upper.setLED(0, 255, 0);
                else s_Upper.setLED(0, 0, 0);
                break;
            case SHOOT:
                intakeSpeed = UpperConstants.INTAKE_SHOOT_SPEED;
                shooterSpeed = UpperConstants.SHOOTER_SHOOT_SPEED;
                s_Upper.blink(0,255,0);
                break;
            case TELE:
                shooterSpeed = 0;
                intakeSpeed = 0;
                s_Upper.setLED(100, 100, 200);
                break;
            case ENDGAME:
                elbowAngle = UpperConstants.ELBOW_GROUND_POS;
                intakeSpeed = 0;
                shooterSpeed = 0;
                s_Upper.gayPride();
                break;
        }

        s_Upper.setElbow(-elbowPID.calculate(elbowAngle - s_Upper.getElbowRotation()));
        s_Upper.setShooter(shooterSpeed);
        s_Upper.setIntake(intakeSpeed);

        SmartDashboard.putString("UpperState", Constants.state.toString());
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
