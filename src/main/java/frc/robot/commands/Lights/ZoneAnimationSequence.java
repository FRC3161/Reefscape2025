package frc.robot.commands.Lights;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lights.LEDSubsystem;

public class ZoneAnimationSequence extends Command {
    private final LEDSubsystem ledSubsystem;
    private final String zoneID;
    private final String sequenceID;
    public ZoneAnimationSequence(LEDSubsystem subsystem, String zoneID, String sequenceID) {
        this.ledSubsystem = subsystem;
        this.zoneID = zoneID;
        this.sequenceID = sequenceID;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        ledSubsystem.LED_GroupAnimationSequence(zoneID, sequenceID);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}