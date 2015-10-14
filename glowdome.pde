/*

 Glow Dome
 
 Libraries used:
 
 pixelpusher 
 http://forum.heroicrobotics.com/board/6/updates
 
 leap motion for processing 
 https://github.com/voidplus/leap-motion-processing
 
 open kinect
 http://shiffman.net/p5/kinect/
 
 Shapes 3D
 http://www.lagers.org.uk/s3d4p/index.html
 
 Keys Used:
 
 w,s: adjust speed in x direction (image layer only)
 e,d: adjust speed in y direction
 p,l: adjust stripe width in stripe layer
 o: reset speed to zero in image layer
 z,x: adjust trace speed, for adjusting POV timing
 i: cycle image used in image layer
 
 number keys: toggle layers
 1: image
 2: stripes
 3: perlin noise
 4: kinect point cloud
 5: sphere
 6: kinect
 */


import org.openkinect.*;
import org.openkinect.processing.*;

import com.heroicrobot.dropbit.registry.*;
import com.heroicrobot.dropbit.devices.pixelpusher.Pixel;
import com.heroicrobot.dropbit.devices.pixelpusher.Strip;

import de.voidplus.leapmotion.*;

import java.util.*;

import processing.serial.*;

String arduinoData = null;  
int lf = 10;    // Linefeed in ASCII

Kinect kinect;

float speedIncrement = 1;

DeviceRegistry registry;

GlowdomeRender sketch;

Serial rpmReader;

boolean useKinect = false;
boolean useLeap = true;

boolean interlaceColumns = false;

int numStripsOverride = 1;
float RPS = 5;//60 FPS
float frameRateVal = RPS * 60;
int last = 0;
float incFactor = 3;

class TestObserver implements Observer {
  public boolean hasStrips = false;
  public void update(Observable registry, Object updatedDevice) {
    println("Registry changed!");
    if (updatedDevice != null) {
      println("Device change: " + updatedDevice);
    }
    this.hasStrips = true;
  }
}


void setup() {
  int stripsHeight = 360;
  
  size(stripsHeight/5, stripsHeight, P3D);
  frameRate(frameRateVal);

  sketch = new GlowdomeRender(this, useKinect, useLeap);
  sketch.setup();

  println(Serial.list());

  if (Serial.list().length > 6) {  
    //rpmReader = new Serial(this, 1Serial.list()[Serial.list().length-1], 152000);
  }
}

/*
    Main draw function pass to sketch object for the actual rendering.
 Handle key presses when we need to detect which keys are held down.
 */
void draw()  {

  if (keyPressed) {
    //println(keyPressed);
        //println(key);
        println(sketch.xSpeed);
        println(sketch.ySpeed);
    switch(key) {
      // adjust speed in x
    case 'w':
      sketch.xSpeed += speedIncrement;
      break;
    case 's':
      sketch.xSpeed -= speedIncrement;
      break;
      // adjust speed in y
    case 'e':
      sketch.ySpeed += speedIncrement;
      break;
    case 'd':
      sketch.ySpeed -= speedIncrement;
      break;
      // adjust stripe width
    case 'p':
      sketch.stripeWidth++;
      break;
    case 'l':
      sketch.stripeWidth--;
      break;
      // adjust trace speed
    case 'z':
      sketch.traceSpeed -= incFactor;
      if (sketch.traceSpeed < 0)
        sketch.traceSpeed = 0;
      break;
    case 'x':
      sketch.traceSpeed += incFactor;
     if (sketch.traceSpeed > 100)
        sketch.traceSpeed = 100;
      break;
    }

    if (sketch.stripeWidth < 2) sketch.stripeWidth = 2;
  }

  sketch.render();
  sketch.display();
}
/*2
 Handle a key press when holding key down is not needed
 */
void keyPressed() {
  switch(key) {
  case 'u'://back
    sketch.cycleImage(-1);
    break;
  case 'i'://forward
    sketch.cycleImage(1);
    break;
  case 'o':
    sketch.resetSpeed();
    break;
  case '0':
    sketch.clearLayers();
    break;
  case 'n':
    interlaceColumns = !interlaceColumns;
    println(interlaceColumns);
    break;
  case RETURN:
  case ENTER:
    sketch.toggleTextEntry();
    break;
  }

  if (key >= '1' && key <= '9') {
    sketch.toggleLayer(key - '0');
  } else if (sketch.textEntry && key >= 'a' && key <= 'z') {
    sketch.sendKey(key);
  }
}

void movieEvent(Movie m) {
  m.read();
}


void stop() {
  sketch.stop();
  super.stop();
}


void leapOnSwipeGesture(SwipeGesture g, int state) {
  int     id                  = g.getId();
  Finger  finger              = g.getFinger();
  PVector position            = g.getPosition();
  PVector position_start      = g.getStartPosition();
  PVector direction           = g.getDirection();
  float   speed               = g.getSpeed();
  long    duration            = g.getDuration();
  float   duration_seconds    = g.getDurationInSeconds();

  switch(state) {
  case 1: // Start
    break;
  case 2: // Update
    break;
  case 3: // Stop
    println("SwipeGesture: " + id);
    //sketch.cycleMode();
    break;
  }
}



void serialEvent(Serial rpmReader) {
  arduinoData = trim(rpmReader.readStringUntil(lf));
  if (arduinoData != null) {
    print("In:");
    println(arduinoData);
    String[] results = match(arduinoData, "\\d\\.\\d+");
    if (results != null) {  
      //float f = Float.parseFloat(arduinoData);
      //println(f);
      //frameRate(f);
    }
  }
}





