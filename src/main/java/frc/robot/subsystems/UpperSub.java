package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.UpperConstants;
import frc.robot.Constants.robotConstants;

public class UpperSub extends SubsystemBase{
        
    private final TalonFX leftElbow = new TalonFX(UpperConstants.leftElbowMotorID, robotConstants.canbusName);
    private final TalonFX rightElbow = new TalonFX(UpperConstants.rightElbowMotorID, robotConstants.canbusName);
    
    private final CANSparkMax leftShooter = new CANSparkMax(UpperConstants.leftShooterMotorID, MotorType.kBrushless);
    private final CANSparkMax rightShooter = new CANSparkMax(UpperConstants.rightShooterMotorID, MotorType.kBrushless);

    private final CANSparkMax intake = new CANSparkMax(UpperConstants.intakeMotorID, MotorType.kBrushless);

    private final CANcoder elbowCancoder = new CANcoder(UpperConstants.elbowCancoderID, robotConstants.canbusName);

    private final AddressableLED led = new AddressableLED(UpperConstants.ledPwmPort);
    private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(UpperConstants.ledLength);

    private final DigitalInput leftLimitSwitch = new DigitalInput(UpperConstants.LeftLimitSwitchID);
    private final DigitalInput rightLimitSwitch = new DigitalInput(UpperConstants.rightLimitSwitchID);

    Timer timer = new Timer();

    public UpperSub() {
        leftElbow.setInverted(false);
        rightElbow.setInverted(true);

        leftShooter.setInverted(false);
        rightShooter.setInverted(false);

        configElbow();

        led.setLength(UpperConstants.ledLength);
        setLED(0,0,0);
        led.start();
    }

    // config
    public void configElbow() {
        elbowCancoder.getConfigurator().apply(
            new CANcoderConfiguration().MagnetSensor.withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf)
        );
    }

    public void configShooter() {
        leftShooter.restoreFactoryDefaults();
        rightShooter.restoreFactoryDefaults();
        leftShooter.setIdleMode(IdleMode.kCoast);
        rightShooter.setIdleMode(IdleMode.kCoast);
    }

    // elbow
    public double getElbowRotation() {
        return elbowCancoder.getPosition().getValue();
    }

    public void setElbow(double speed) {
        leftElbow.set(speed);
        rightElbow.set(speed);
    }

    // public double calculateElbow(double distance) {
    //     double down0 = 0,down1 = 0,up0 = 0.2,up1 = 0;
    //     for(int i=0;i<VisionConstants.data.length-1;i++) {
    //         if(VisionConstants.data[i][0] > distance) {
    //             up0 = VisionConstants.data[i][0];
    //             up1 = VisionConstants.data[i][1];
    //             down0 = VisionConstants.data[i-1][0];
    //             down1 = VisionConstants.data[i-1][1];
    //             break;
    //         }
    //     }
    //     return ((distance - down0)*down1 + (up0-distance)*up1) / (up0 - down0);
    // };

    // intake
    public double getIntakeVel() {
        return intake.getEncoder().getVelocity();
    }

    public void setIntake(double speed) {
        intake.set(speed);
    }

    // shooter
    public double getLeftShooterRPM() {
        return leftShooter.getEncoder().getVelocity();
    }

    public double getRightShooterRPM() {
        return rightShooter.getEncoder().getVelocity();
    }

    public double getShooterRPM() {
        return (getLeftShooterRPM() + getRightShooterRPM()) / 2;
    }

    public void setShooter(double speed) {
        leftShooter.set(speed);
        rightShooter.set(speed * (Constants.UpperConstants.leftShooterRPM / Constants.UpperConstants.rightShooterRPM)); // brute force attack
    }

    // LED
    public void setLED(int r, int g, int b) {
        for(int i=0;i<buffer.getLength()-1;i++) buffer.setRGB(i, r, g, b);
        led.setData(buffer);
    }

    public void blink(int r, int g, int b) {
        timer.start();
        if(timer.get() < 0.1) setLED(r, g, b);
        else if(timer.get() < 0.2) setLED(0, 0, 0);
        else timer.restart();
    }

    public void gayPride() {
        int counter = 0;
        boolean oneTime = false;
        timer.start();
        if(timer.get() < 0.1) {
            oneTime = false;
            for(int i=0;i<buffer.getLength();i++) {
                if(i%6 == (5+counter)%6)  buffer.setRGB(i, 255, 0, 255);
                else if (i%6 == (0+counter)%6)  buffer.setRGB(i, 255, 0, 0);
                else if (i%6 == (1+counter)%6)  buffer.setRGB(i, 255, 255, 0);
                else if (i%6 == (2+counter)%6)  buffer.setRGB(i, 0, 255, 0);
                else if (i%6 == (3+counter)%6)  buffer.setRGB(i, 0, 255, 255);
                else if (i%6 == (4+counter)%6)  buffer.setRGB(i, 0, 0, 255);
            }
        } else if (timer.get() >= 0.1 && !oneTime) {
            counter++;
            oneTime = true;
        } else timer.restart();
    }

    // limitSwitch
    public boolean hasNote() {
        return (leftLimitSwitch.get() || rightLimitSwitch.get());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("elbowDEG", getElbowRotation());
        SmartDashboard.putNumber("ShooterRPM", getShooterRPM());
        SmartDashboard.putBoolean("hasNote", hasNote());
    }
}   