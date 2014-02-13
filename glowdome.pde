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

Kinect kinect;

DeviceRegistry registry;

GlowdomeRender sketch;

Serial rpmReader;

boolean useKinect = false;
boolean useLeap = true;

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
    size(480, 480, P3D);
    frameRate(120);

    sketch = new GlowdomeRender(this, useKinect, useLeap);
    sketch.setup();

    println(Serial.list());
    rpmReader = new Serial(this, Serial.list()[0], 9600);
}

/*
    Main draw function pass to sketch object for the actual rendering.
    Handle key presses when we need to detect which keys are held down.
 */
void draw()  {

    float speedIncrement = 0.2;

    if (keyPressed) {

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
                sketch.traceSpeed -= 0.1;
                if (sketch.traceSpeed < 0)
                    sketch.traceSpeed = 0;
                break;
            case 'x':
                sketch.traceSpeed += 0.1;

                break;
        }

        if (sketch.stripeWidth < 2) sketch.stripeWidth = 2;
    }

    sketch.render();
    sketch.display();

}


/*
    Handle a key press when holding key down is not needed
 */
void keyPressed() {
    switch(key) {
        case 'i':
            sketch.cycleImage();
            break;
        case 'o':
            sketch.resetSpeed();
            break;
        case '0':
            sketch.clearLayers();
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

void serialEvent(Serial port) {
    int val = port.read();

    println("raw input" + val);
}

void stop() {
    sketch.stop();
    super.stop();
}


void leapOnSwipeGesture(SwipeGesture g, int state){
    int     id                  = g.getId();
    Finger  finger              = g.getFinger();
    PVector position            = g.getPosition();
    PVector position_start      = g.getStartPosition();
    PVector direction           = g.getDirection();
    float   speed               = g.getSpeed();
    long    duration            = g.getDuration();
    float   duration_seconds    = g.getDurationInSeconds();

    switch(state){
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




