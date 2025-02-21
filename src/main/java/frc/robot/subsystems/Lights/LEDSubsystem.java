package frc.robot.subsystems.Lights;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Map;

//Imports
//import edu.wpi.first.units.Units.*;
//import edu.wpi.first.units.measure.Distance;
//import edu.wpi.first.math.util.Units;
//import edu.wpi.first.units.TimeUnit;
import static edu.wpi.first.units.Units.Milliseconds;
//import static edu.wpi.first.units.Units.Meters;
//import static edu.wpi.first.units.Units.MetersPerSecond;
//import static edu.wpi.first.units.Units.Seconds;

//LED Imports
import com.lumynlabs.devices.ConnectorXAnimate;
import com.lumynlabs.domain.led.Animation;
import com.lumynlabs.domain.led.MatrixTextScrollDirection;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.util.Color;

//Main Class
public class LEDSubsystem extends SubsystemBase {

  // Setup
  private ConnectorXAnimate cXAnimate = new ConnectorXAnimate(); // LED Controller
  private Map<String, Color> colors = Map.of( // Color Map
      "black", new Color(0, 0, 0),
      "white", new Color(255, 255, 255),
      "red", new Color(255, 0, 0),
      "green", new Color(0, 255, 0),
      "blue", new Color(0, 0, 255),
      "team_Gold", new Color(179, 134, 27));

  // LED Strip Start
  public LEDSubsystem() {
    cXAnimate.Connect(Port.kUSB1); // Connect to LED Controller
    setDefaultCommand(LED_Reset().withName("LED_Reset"));
  }

  /**
   * Resetting LED  - LED set to solid black.
   */
  public Command LED_Reset() {
    return run(() -> cXAnimate.leds.SetGroupColor("", colors.get("black"))); // Total Group
  }

  /**
   * Solid group color.
   * 
   * @param groupID the group to set the color
   * @param color   the color of the pattern
   */
  public void LED_GroupColor(String groupID, Color color) {
    cXAnimate.leds.SetGroupColor(groupID, color);
  }

  /**
   * Solid zone color.
   * 
   * @param zoneID the zone to set the color
   * @param color  the color of the pattern
   */
  public void LED_ZoneColor(String zoneID, Color color) {
    cXAnimate.leds.SetGroupColor(zoneID, color);
  }

  /**
   * Group Animation.
   * 
   * @param groupID   the group to set the pattern
   * @param animation the animation to run
   * @param color     the color of the pattern
   * @param delay     the delay of the pattern
   * @param reversed  the direction of the pattern
   * @param oneShot   the pattern to run once
   */
  public void LED_GroupAnimation(String groupID, Animation animation, Color color, int delay, boolean reversed,
      boolean oneShot) {
    cXAnimate.leds.SetGroupAnimation(groupID, animation, color, Milliseconds.of(delay), reversed, oneShot);
  }

  /**
   * Zone Animation.
   * 
   * @param zoneID    the group to set the pattern
   * @param animation the animation to run
   * @param color     the color of the pattern
   * @param delay     the delay of the pattern
   * @param reversed  the direction of the pattern
   * @param oneShot   the pattern to run once
   */
  public void LED_ZoneAnimation(String zoneID, Animation animation, Color color, int delay, boolean reversed,
      boolean oneShot) {
    cXAnimate.leds.SetAnimation(zoneID, animation, color, Milliseconds.of(delay), reversed, oneShot);
  }

  /**
   * Group Animation Sequence.
   * 
   * @param groupID    the group to set the pattern
   * @param sequenceID the sequence to run
   */
  public void LED_GroupAnimationSequence(String groupID, String sequenceID) {
    cXAnimate.leds.SetGroupAnimationSequence(groupID, sequenceID);
  }

  /**
   * Zone Animation Sequence.
   * 
   * @param zoneID     the group to set the pattern
   * @param sequenceID the sequence to run
   */
  public void LED_ZoneAnimationSequence(String zoneID, String sequenceID) {
    cXAnimate.leds.SetAnimationSequence(zoneID, sequenceID);
  }

  /**
   * Group Image Sequence.
   * 
   * @param groupID    the group to set the image sequence
   * @param sequenceID the sequence to run
   * @param color      the color of the sequence
   * @param setColor   whether to set the color of the sequence
   * @param oneShot    whether the sequence should run once
   */
  public void LED_GroupImageSequence(String groupID, String sequenceID, Color color, Boolean setColor,
      boolean oneShot) {
    cXAnimate.leds.SetGroupImageSequence(groupID, sequenceID, color, setColor, oneShot);
  }

  /**
   * Zone Image Sequence.
   * 
   * @param zoneID     the zone to set the image sequence
   * @param sequenceID the sequence to run
   * @param color      the color of the sequence
   * @param setColor   whether to set the color of the sequence
   * @param oneShot    whether the sequence should run once
   */
  public void LED_ZoneImageSequence(String zoneID, String sequenceID, Color color, Boolean setColor, boolean oneShot) {
    cXAnimate.leds.SetImageSequence(zoneID, sequenceID, color, setColor, oneShot);
  }

  /**
   * Group Matrix Text.
   * 
   * @param groupID   the group to set the text
   * @param text      the text to display
   * @param color     the color of the text
   * @param direction the direction of the text scroll
   * @param delay     the delay between scrolls in milliseconds
   * @param oneShot   whether the text should scroll once
   */
  public void LED_GroupMatrixText(String groupID, String text, Color color, MatrixTextScrollDirection direction,
      int delay, boolean oneShot) {
    cXAnimate.leds.SetGroupMatrixText(groupID, text, color, direction, Milliseconds.of(delay), oneShot);
  }

  /**
   * Zone Matrix Text.
   * 
   * @param zoneID    the zone to set the text
   * @param text      the text to display
   * @param color     the color of the text
   * @param direction the direction of the text scroll
   * @param delay     the delay between scrolls in milliseconds
   * @param oneShot   whether the text should scroll once
   */
  public void LED_ZoneMatrixText(String zoneID, String text, Color color, MatrixTextScrollDirection direction,
      int delay, boolean oneShot) {
    cXAnimate.leds.SetMatrixText(zoneID, text, color, direction, Milliseconds.of(delay), oneShot);
  }

  @Override
  public void periodic() {
  }
}
