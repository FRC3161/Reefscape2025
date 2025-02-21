package frc.robot.commands.Lights;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lights.LEDSubsystem;
import com.lumynlabs.domain.led.Animation;
import edu.wpi.first.wpilibj.util.Color;

public class GroupAnimation extends Command {
    private final LEDSubsystem ledSubsystem;
    private final String groupID;
    private final Animation animation;
    private final Color color;
    private final int delay;
    private final boolean reversed;
    private final boolean oneShot;

    public GroupAnimation(LEDSubsystem subsystem, String groupID, Animation animation, Color color, int delay, boolean reversed, boolean oneShot) {
        this.ledSubsystem = subsystem;
        this.groupID = groupID;
        this.animation = animation;
        this.color = color;
        this.delay = delay;
        this.reversed = reversed;
        this.oneShot = oneShot;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        ledSubsystem.LED_GroupAnimation(groupID, animation, color, delay, reversed, oneShot);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}