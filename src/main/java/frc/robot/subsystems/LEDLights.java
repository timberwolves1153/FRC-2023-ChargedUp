package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class LEDLights extends SubsystemBase {
  private AddressableLED m_led;
  private AddressableLED m_led2;
  private AddressableLEDBuffer m_ledBuffer;
  private int m_rainbowFirstPixelHue;

  private Color[] chaseColors = {new Color(255/255.0, 20/255.0, 147/255.0)};

  /** Creates a new LEDLights. */
    public LEDLights() {
    m_led = new AddressableLED(0);
    m_ledBuffer = new AddressableLEDBuffer(20);
    m_led.setLength(m_ledBuffer.getLength());

    m_led.start();
    }

    public void setRGB(int r, int g, int b) {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, r, g, b);
    }

    m_led.setData(m_ledBuffer);
    }

    public void setYellow(){
    m_ledBuffer.setRGB(20, 255, 255, 0);
    System.out.println("Yellow running");
    }

    public void setPurple(){
    m_ledBuffer.setRGB(20, 128, 0, 128);
    System.out.println("Purple running");
    }

    public void ledStop(){
    m_ledBuffer.setRGB(20, 0, 0, 0);
    }

//   public void setLEDPattern(TrobotAddressableLEDPattern pattern) {
//     pattern.setLEDs(m_ledBuffer);
//   }

  private void rainbow() {
    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
      // Set the value
      m_ledBuffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
  }

  @Override
  public void periodic() {
  }
}

