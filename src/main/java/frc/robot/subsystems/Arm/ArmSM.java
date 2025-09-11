
//Imports
package frc.robot.subsystems.Arm;

import java.lang.Thread.State;
import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.LoggedTunableNumber;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm.Encoders.ArmEncoderThroughbore;
import frc.robot.subsystems.Intake.Intake;

public class ArmSM extends SubsystemBase {
  private enum WantedState {
    MANUAL,
    INTAKE,
    L1,
    L2_CORAL,
    L2_ALGAE,
    L3_ALGAE,
    L3_CORAL_BATTERY,
    L3_CORAL_PIVOT,
    L4_CORAL_BATTERY,
    L4_CORAL_PIVOT,
    IDLE,
    ALGAE_BARGE,
    OFF
  }

  private enum SystemState {
    MANUAL_MODE,
    INTAKING,
    MOVING_L1,
    MOVING_L2_CORAL,
    MOVING_L2_ALGAE,
    MOVING_L3_ALGAE,
    MOVING_L3_CORAL_BATTERY,
    MOVING_L3_CORAL_PIVOT,
    MOVING_L4_CORAL_BATTERY,
    MOVING_L4_CORAL_PIVOT,
    IDLING,
    GOING_ALGAE_BARGE,
    OFF
  }

  // Initialize motors
  private final SparkMax leader = new SparkMax(Constants.ArmConstants.leaderID, MotorType.kBrushless);
  private final SparkMax follower = new SparkMax(Constants.ArmConstants.followerID, MotorType.kBrushless);

  private final SparkMaxConfig armLeaderConfig = new SparkMaxConfig();
  private final SparkMaxConfig armFollowerConfig = new SparkMaxConfig();

  // Initialize encoders
  private final ArmEncoderThroughbore encoder = new ArmEncoderThroughbore(Constants.ArmConstants.encoderID);
  public static Rotation2d encoderPosition = new Rotation2d();

  // PID controller + feedforward initialization
  private final ProfiledPIDController pid = new ProfiledPIDController(
            Constants.ArmConstants.armPID[0],
            Constants.ArmConstants.armPID[1],
            Constants.ArmConstants.armPID[2],
            new TrapezoidProfile.Constraints(
              ArmConstants.maxVelocityPerSecond.getRadians(), 
              ArmConstants.maxAcceleration.getRadians()));

  private ArmFeedforward ffModel = new ArmFeedforward(
      Constants.ArmConstants.armSGV[0],
      Constants.ArmConstants.armSGV[1],
      Constants.ArmConstants.armSGV[2]);

  // Motion profiling
  private Rotation2d setpoint = new Rotation2d();
  private Rotation2d velocity = new Rotation2d();
  private Rotation2d goal = new Rotation2d();

  // States
  SystemState systemState = SystemState.OFF;
  WantedState wantedState = WantedState.OFF;

  Intake intake = new Intake();
  private RobotContainer m_Container = new RobotContainer();
  // Tunable values
  // private LoggedTunableNumber armP = new LoggedTunableNumber("armP",
  // Constants.ArmConstants.armPID[0]);
  // private LoggedTunableNumber armI = new LoggedTunableNumber("armI",
  // Constants.ArmConstants.armPID[1]);
  // private LoggedTunableNumber armD = new LoggedTunableNumber("armD",
  // Constants.ArmConstants.armPID[2]);
  // private LoggedTunableNumber armS = new LoggedTunableNumber("armS",
  // Constants.ArmConstants.armSGV[0]);
  // private LoggedTunableNumber armG = new LoggedTunableNumber("armG",
  // Constants.ArmConstants.armSGV[1]);
  // private LoggedTunableNumber armV = new LoggedTunableNumber("armV",
  // Constants.ArmConstants.armSGV[2]);

  public ArmSM() {
    setupMotors();
    // Set offset + get encoders position
    encoder.setOffset(Constants.ArmConstants.offset);
    encoderPosition = encoder.getAbsolutePosition();
    // runSetpoint(getEncoderPosition());
  }

  // Motor Set-Up
  private void setupMotors() {

    armLeaderConfig.voltageCompensation(12).smartCurrentLimit(50, 30).inverted(true).idleMode(IdleMode.kBrake);
    leader.configure(armLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    CANSparkMaxUtil.setSparkMaxBusUsage(leader, armLeaderConfig, Usage.kPositionOnly);

    CANSparkMaxUtil.setSparkMaxBusUsage(follower, armFollowerConfig, Usage.kPositionOnly);
    armFollowerConfig.voltageCompensation(12).smartCurrentLimit(50, 30).inverted(false).idleMode(IdleMode.kBrake);
    follower.configure(armFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    pid.setIntegratorRange(-0.05, 0.05);
  }

  // Resets the PID's i value
  public void resetI() {
    pid.reset(new TrapezoidProfile.State(getEncoderPosition().getRadians(), velocity.getRadians()));
  }

  // returns the encoder's position
  public static Rotation2d getEncoderPosition() {
    return encoderPosition;
  }

  // Checks to see if the PID and SGV values have changed.
  // Updates the value if it's been changed
  // public void checkTunableValues() {
  // // if (!Constants.enableTunableValues)
  // // return;

  // if (armP.hasChanged() || armI.hasChanged() || armD.hasChanged()) {
  // pid.setPID(armP.get(), armI.get(), armD.get());
  // }

  // if (armS.hasChanged() || armG.hasChanged() || armV.hasChanged()) {
  // ffModel = new ArmFeedforward(armS.get(), armG.get(), armV.get());
  // }
  // }

  // Logs values to SmartDashboard/Glass
  private void logValues() {
    SmartDashboard.putNumber("Arm Actual Angle", getEncoderPosition().getDegrees());
    SmartDashboard.putNumber("Arm Desired Angle", setpoint.getDegrees());
    SmartDashboard.putNumber("Arm Desired Speed", velocity.getDegrees());
    SmartDashboard.putNumber("Arm current", leader.getOutputCurrent());
    SmartDashboard.putNumber("arm output", leader.getAppliedOutput());
  }

  public void runSetpoint(Rotation2d setpoint) {
    if (setpoint.getRadians() < Constants.ArmConstants.min.getRadians()) {
      this.setpoint = Constants.ArmConstants.min;
    } else if (setpoint.getRadians() > Constants.ArmConstants.max.getRadians()) {
      this.setpoint = Constants.ArmConstants.max;
    } else {
      this.setpoint = setpoint;
    }
  }

  private SystemState changeCurrentState() {
    return switch (wantedState) {
      case MANUAL:
        yield SystemState.MANUAL_MODE;
      case INTAKE:
        if (!intake.hasCoral()) {
          yield SystemState.INTAKING;
        } else {
          yield SystemState.IDLING;
        }
      case L1:
        if (intake.hasCoral()) {
          yield SystemState.MOVING_L1;
        } else {
          yield SystemState.IDLING;
        }
      case L2_CORAL:
        if (intake.hasCoral()) {
          yield SystemState.MOVING_L2_CORAL;
        } else {
          yield SystemState.IDLING;
        }
      case L2_ALGAE:
        yield SystemState.MOVING_L2_ALGAE;
      case L3_ALGAE:
        yield SystemState.MOVING_L3_ALGAE;
      case L3_CORAL_BATTERY:
        if (intake.hasCoral()) {
          yield SystemState.MOVING_L3_CORAL_BATTERY;
        } else {
          yield SystemState.IDLING;
        }
      case L3_CORAL_PIVOT:
        if (intake.hasCoral()) {
          yield SystemState.MOVING_L3_CORAL_PIVOT;
        } else {
          yield SystemState.IDLING;
        }
      case L4_CORAL_BATTERY:
        if (intake.hasCoral()) {
          yield SystemState.MOVING_L4_CORAL_BATTERY;
        } else {
          yield SystemState.IDLING;
        }
      case L4_CORAL_PIVOT:
        if (intake.hasCoral()) {
          yield SystemState.MOVING_L4_CORAL_PIVOT;
        } else {
          yield SystemState.IDLING;
        }
      case IDLE:
        yield SystemState.IDLING;
      case ALGAE_BARGE:
        yield SystemState.GOING_ALGAE_BARGE;
      case OFF:
        yield SystemState.OFF;
    };
  }

  private void applyState() {
    
    switch (systemState) {
      case MANUAL_MODE:
        double armY = -m_Container.operator.getLeftY();
        runSetpoint(new Rotation2d(Units.degreesToRadians(armY + setpoint.getDegrees())));
      case INTAKING:
        setpoint = new Rotation2d(ArmConstants.INTAKE_SETPOINT);
      case MOVING_L1:
        setpoint = new Rotation2d(ArmConstants.L1_CORAL_SETPOINT);
      case MOVING_L2_CORAL:
        setpoint = new Rotation2d(ArmConstants.L2_CORAL_SETPOINT);
      case MOVING_L2_ALGAE:
        setpoint = new Rotation2d(ArmConstants.L2_ALGAE_SETPOINT);
      case MOVING_L3_ALGAE:
        setpoint = new Rotation2d(ArmConstants.L3_ALGAE_SETPOINT);
      case MOVING_L3_CORAL_BATTERY:
        setpoint = new Rotation2d(ArmConstants.L3_CORAL_BATTERY_SETPOINT);
      case MOVING_L3_CORAL_PIVOT:
        setpoint = new Rotation2d(ArmConstants.L3_CORAL_PIVOT_SETPOINT);
      case MOVING_L4_CORAL_BATTERY:
        setpoint = new Rotation2d(ArmConstants.L4_CORAL_BATTERY_SETPOINT);
      case MOVING_L4_CORAL_PIVOT:
        setpoint = new Rotation2d(ArmConstants.L3_CORAL_PIVOT_SETPOINT);
      case IDLING:
        setpoint= new Rotation2d(ArmConstants.IDLE_SETPOINT);
      case GOING_ALGAE_BARGE:
        setpoint = new Rotation2d(ArmConstants.ALGAE_BARGE_SETPOINT);
      case OFF:
    }
  }

  @Override
  public void periodic() {
    systemState = changeCurrentState();

    encoderPosition = encoder.getAbsolutePosition(); // Finds the exact position of the encoder
    logValues(); // Logs values to SmartDashboard/Glass
    // checkTunableValues(); //Updates PID and FF values

    var ffOutput = ffModel.calculate(setpoint.getRadians(), velocity.getRadians()); // Calculates Feedforward output
    var pidOutput = pid.calculate(getEncoderPosition().getRadians(), setpoint.getRadians()); // calculates PID output

    // SmartDashboard.putNumber("ffoutput arm", ffOutput); //Displays the FF output
    // calculated above on Smartdahsboard/Glass

    // PID+FF output on the leader and follower motors
    leader.set(ffOutput + pidOutput);
    follower.set(ffOutput + pidOutput);
    // follower.set(ffOutput + pidOutput);
  }

}
