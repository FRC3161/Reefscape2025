package frc.robot.commands.Lights;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lights.LEDSubsystem;
import edu.wpi.first.wpilibj.util.Color;

public class GroupImageSequence extends Command {
    private final LEDSubsystem ledSubsystem;
    private final String groupID;
    private final String sequenceID;
    private final Color color;
    private final boolean setColor;
    private final boolean oneShot;

    public GroupImageSequence(LEDSubsystem subsystem, String groupID, String sequenceID, Color color, boolean setColor, boolean oneShot) {
        this.ledSubsystem = subsystem;
        this.groupID = groupID;
        this.sequenceID = sequenceID;
        this.color = color;
        this.setColor = setColor;
        this.oneShot = oneShot;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        ledSubsystem.LED_GroupImageSequence(groupID, sequenceID, color, setColor, oneShot);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}