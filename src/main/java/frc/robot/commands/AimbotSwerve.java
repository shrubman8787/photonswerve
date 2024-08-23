package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.ChenryLib.PID;
import frc.robot.Constants;
import frc.robot.Constants.LimeLight;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.VisionSub;

public class AimbotSwerve extends Command {
  private Swerve s_Swerve;
  private VisionSub s_Vision;
  private XboxController driver;

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

  private double translationVal;
  private double strafeVal;
  private double rotationVal;
  private double ema;

  public double KP = Constants.LimeLight.KPDefault;
  public double KI = Constants.LimeLight.KIDefault;
  public double KD = Constants.LimeLight.KDDefault;
  public double WindUp = Constants.LimeLight.WindupDefault;
  public double Limit = Constants.LimeLight.LimitDefault;
  public double Smooth = Constants.LimeLight.SmoothDefault;

  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  
  private PID facingPID = new PID(KP, KI, KD, WindUp, Limit);

  // Constants such as camera and target height stored. Change per robot and goal!
  final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(8);
  final double TARGET_HEIGHT_METERS = Units.inchesToMeters(54);
  // Angle between horizontal and the camera.
  final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(60);
  //chage the name accordingly
  PhotonCamera camera = new PhotonCamera("photonvision");

  public AimbotSwerve(Swerve s_Swerve,VisionSub s_Vision, XboxController controller) {
    this.s_Swerve = s_Swerve;
    this.s_Vision = s_Vision;
    this.driver = controller;
    addRequirements(s_Swerve);
    addRequirements(s_Vision);
  }

  @Override
  public void execute() {
    SmartDashboard.putNumber("speed", rotationVal);

    var vision = camera.getLatestResult();

    camera.setLED(VisionLEDMode.kBlink);
    
    /* Get Values, Deadband */
    translationVal = translationLimiter.calculate(
        MathUtil.applyDeadband(driver.getLeftY(), 0.01));
    strafeVal = strafeLimiter.calculate(
        MathUtil.applyDeadband(driver.getLeftX(), 0.01));
    
    // if (Math.abs(tx.getDouble(0.0))<=1) {
    //   ema = Smooth*tx.getDouble(0.0)+(1-Smooth)*ema;
    //   rotationVal = ema*1;
    //   // rotationVal=0;
    // }else{
    //   rotationVal = facingPID.calculate(tx.getDouble(0.0))*1;
    //   // rotationVal = MathUtil.applyDeadband(tx.getDouble(0), 0.05);
    // }

    // rotationVal = facingPID.calculate(tx.getDouble(0.0));
    double range =PhotonUtils.calculateDistanceToTargetMeters(
                      CAMERA_HEIGHT_METERS,
                      TARGET_HEIGHT_METERS,
                      CAMERA_PITCH_RADIANS,
                      Units.degreesToRadians(vision.getBestTarget().getPitch()));

    SmartDashboard.putNumber("range", range);


    ema = Smooth*facingPID.calculate(vision.getBestTarget().getYaw())+(1-Smooth)*ema;
    // rotationVal = ema;
    SmartDashboard.putNumber("output", ema);

    if (driver.getBackButton()) {
      s_Swerve.zeroGyro(); 
      s_Swerve.resetPose(new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));
    }

    /* Drive */
    s_Swerve.drive(
        new Translation2d(translationVal, strafeVal).times(Constants.SwerveConstants.maxSpeed),
        rotationVal * Constants.SwerveConstants.maxAngularVelocity, true,
        true);
  }

  @Override
  public void end(boolean interrupted){
    System.out.println("bro ur teleop swerve is fucked");
  }

  @Override
  public boolean isFinished(){
    return false;
  }
}