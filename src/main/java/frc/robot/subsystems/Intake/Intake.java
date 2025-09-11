package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Intake extends SubsystemBase {

    private enum WantedState {
        SCORE_L1,
        SCORE_BATTERY_SIDE,
        SCORE_PIVOT_SIDE,
        SCORE_ALGAE,
        INTAKE,
        OFF
    }

    private enum SystemState {
        SCORING_L1,
        SCORING_BATTERY_SIDE,
        SCORING_PIVOT_SIDE,
        SCORING_ALGAE,
        INTAKING,
        OFF
    }

    private static TalonFX intake = new TalonFX(61);

    private static MotorOutputConfigs intakeConfig = new MotorOutputConfigs();

    private static DigitalInput beamBreak = new DigitalInput(1);

    private WantedState wantedState = WantedState.OFF;

    private SystemState systemState = SystemState.OFF;

    private static Timer pulseTimer = new Timer();
    public static boolean modified = false;

    public Intake() {
        setupMotors();
        pulseTimer.reset();
    }

    public boolean hasCoral() {
        return !beamBreak.get();
    }

    public void setupMotors() {
        // Apply Configs
        intake.setNeutralMode(NeutralModeValue.Brake);
        intakeConfig.Inverted = InvertedValue.Clockwise_Positive;
    }

    public void hpIntake(double value) {
        if (beamBreak.get()) {
            intake.set(value);
        } else {
            intake.set(0);
        }

    }

    // Outtakes through the black wheels
    public void outTake(double value) {
        intake.set(value);
    }

    public void stop() {
        intake.set(0);
    }

    private SystemState changeCurrentState() {
        return switch (wantedState) {
            case SCORE_L1: {
                yield systemState = systemState.SCORING_L1;
            }
            case SCORE_BATTERY_SIDE: {
                yield systemState = systemState.SCORING_BATTERY_SIDE;
            }
            case SCORE_PIVOT_SIDE: {
                yield systemState = systemState.SCORING_PIVOT_SIDE;
            }
            case SCORE_ALGAE: {
                yield systemState = systemState.SCORING_ALGAE;
            }
            case INTAKE: {
                yield systemState = systemState.INTAKING;
            }
            case OFF: { 
                yield systemState = systemState.OFF;
            }
            default: {
                yield systemState.OFF;
            }
        }; 
    }

    private void applyState() {
        double motorSpeed = 0.0;

        switch (systemState) {
            case SCORING_L1:
                motorSpeed = .2;
            case SCORING_BATTERY_SIDE:
                motorSpeed = -0.7;
            case SCORING_PIVOT_SIDE:
                motorSpeed = 0.7;
            case SCORING_ALGAE:
                motorSpeed = 0.7;
            case INTAKING:
                if(!hasCoral()) {
                    motorSpeed = -0.4;
                } else {
                    motorSpeed = 0.0;
                }
            case OFF:
                motorSpeed = 0.0;
        }
        intake.set(motorSpeed);    
    }

    public void setWantedState(WantedState desiredState) {
        this.wantedState = desiredState;
    }

    @Override
    public void periodic() {
        systemState = changeCurrentState();
        applyState();
    }
}
