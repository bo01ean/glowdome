import processing.core.*; 
import processing.data.*; 
import processing.event.*; 
import processing.opengl.*; 

import org.openkinect.*; 
import org.openkinect.processing.*; 
import com.heroicrobot.dropbit.registry.*; 
import com.heroicrobot.dropbit.devices.pixelpusher.Pixel; 
import com.heroicrobot.dropbit.devices.pixelpusher.Strip; 
import de.voidplus.leapmotion.*; 
import java.util.*; 
import processing.serial.*; 
import shapes3d.utils.*; 
import shapes3d.animation.*; 
import shapes3d.*; 
import processing.core.PVector; 
import processing.video.*; 
import org.openkinect.*; 
import org.openkinect.processing.*; 

import java.util.HashMap; 
import java.util.ArrayList; 
import java.io.File; 
import java.io.BufferedReader; 
import java.io.PrintWriter; 
import java.io.InputStream; 
import java.io.OutputStream; 
import java.io.IOException; 

public class glowdome extends PApplet {

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















String arduinoData = null;  
int lf = 10;    // Linefeed in ASCII

Kinect kinect;

float speedIncrement = 1;

DeviceRegistry registry;

GlowdomeRender sketch;

Serial rpmReader;

boolean readSerial = true;

boolean useKinect = false;
boolean useLeap = true;

boolean interlaceColumns = false;

int numStripsOverride = 1;
float RPS = 1;//60 FPS
float frameRateVal = RPS * 60;
int last = 0;
float incFactor = 3;

class TestObserver implements Observer {
  public boolean hasStrips = false;
  public void update(Observable registry, Object updatedDevice) {
    //println("Registry changed!");
    if (updatedDevice != null) {
      //println("Device change: " + updatedDevice);
    }
    this.hasStrips = true;
  }
}


public void setup() {
  int stripsHeight = 360;
  
  size(stripsHeight/5, stripsHeight, P3D);
  frameRate(60);

  sketch = new GlowdomeRender(this, useKinect, useLeap);
  sketch.setup();
  //println(Serial.list());
    // check for port
    if(readSerial) {
      String[] serialList = Serial.list();
      for(int i =0; i< serialList.length; i++) {
        println(serialList[i]);
          String results[] = match(serialList[i], "cu\\.usbmodem1059031");
          
          if(results != null) {   
            rpmReader = new Serial(this, "/dev/cu.usbmodem1059031", 9600);
            rpmReader.bufferUntil(lf);
            rpmReader.bufferUntil('\n');        
          }
      }
  }
}

/*
    Main draw function pass to sketch object for the actual rendering.
 Handle key presses when we need to detect which keys are held down.
 */
public void draw()  {

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
public void keyPressed() {
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

public void movieEvent(Movie m) {
  m.read();
}


public void stop() {
  sketch.stop();
  super.stop();
}


public void leapOnSwipeGesture(SwipeGesture g, int state) {
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



public void serialEvent(Serial teensy) {
  
  try {
    arduinoData = trim(teensy.readStringUntil(lf));
    if (arduinoData != null) {
      println("RPM FROM ARDUINO: " + arduinoData);
      println(Integer.parseInt(arduinoData));
      frameRate(Integer.parseInt(arduinoData));
    }
  } catch (Exception e) {
    //println("Noooooo");
  }
}





class KinectTracker {



  // Size of kinect image
  int kw = 640;
  int kh = 480;
  int threshold = 745;

  // Raw location
  PVector loc;

  // Interpolated location
  PVector lerpedLoc;

  // Depth data
  int[] depth;


  PImage display;

  KinectTracker() {
    kinect.start();
    kinect.enableDepth(true);

    // We could skip processing the grayscale image for efficiency
    // but this example is just demonstrating everything
    kinect.processDepthImage(true);

    display = createImage(kw,kh,PConstants.RGB);

    loc = new PVector(0,0);
    lerpedLoc = new PVector(0,0);
  }

  public void track() {

    // Get the raw depth as array of integers
    depth = kinect.getRawDepth();

    // Being overly cautious here
    if (depth == null) return;

    float sumX = 0;
    float sumY = 0;
    float count = 0;

    for(int x = 0; x < kw; x++) {
      for(int y = 0; y < kh; y++) {
        // Mirroring the image
        int offset = kw-x-1+y*kw;
        // Grabbing the raw depth
        int rawDepth = depth[offset];

        // Testing against threshold
        if (rawDepth < threshold) {
          sumX += x;
          sumY += y;
          count++;
        }
      }
    }
    // As long as we found something
    if (count != 0) {
      loc = new PVector(sumX/count,sumY/count);
    }

    // Interpolating the location, doing it arbitrarily for now
    lerpedLoc.x = PApplet.lerp(lerpedLoc.x, loc.x, 0.3f);
    lerpedLoc.y = PApplet.lerp(lerpedLoc.y, loc.y, 0.3f);
  }

  public PVector getLerpedPos() {
    return lerpedLoc;
  }

  public PVector getPos() {
    return loc;
  }

  public void display() {
    PImage img = kinect.getDepthImage();

    // Being overly cautious here
    if (depth == null || img == null) return;

    // Going to rewrite the depth image to show which pixels are in threshold
    // A lot of this is redundant, but this is just for demonstration purposes
    display.loadPixels();
    for(int x = 0; x < kw; x++) {
      for(int y = 0; y < kh; y++) {
        // mirroring image
        int offset = kw-x-1+y*kw;
        // Raw depth
        int rawDepth = depth[offset];

        int pix = x+y*display.width;
        if (rawDepth < threshold) {
          // A red color instead
          display.pixels[pix] = color(150,50,50);
        } 
        else {
          display.pixels[pix] = img.pixels[offset];
        }
      }
    }
    display.updatePixels();

    // Draw the image
    image(display,0,0);
  }

  public void quit() {
    kinect.quit();
  }

  public int getThreshold() {
    return threshold;
  }

  public void setThreshold(int t) {
    threshold =  t;
  }
}





class Glowdome3d {
  
  Ellipsoid earth;
  
  
  public void setup(PApplet applet) {
      earth = new Ellipsoid(applet, 16, 16);
      earth.setTexture("pattern3.png");
      earth.setRadius(180);
      earth.moveTo(new PVector(0, 0, 0));
      earth.strokeWeight(1.0f);
      earth.stroke(color(255, 255, 0));
      earth.moveTo(20, 40, -80);
      earth.tag = "Earth";
      earth.drawMode(Shape3D.TEXTURE);
      
  }  
  
  public void render(PVector handsDelta) {
    pushStyle();
    
    // Change the rotations before drawing
    earth.rotateBy(radians(handsDelta.y), radians(handsDelta.x), 0);
  
    //background(0);
    pushMatrix();
    camera(0, -190, 350, 0, 0, 0, 0, 1, 0);
    lights();
  
    // Draw the earth (will cause all added shapes
    // to be drawn i.e. the moon)
    earth.draw();
  
    //stars.draw();
    popMatrix();
    popStyle();
  }
  
  
}








class GlowdomeRender {
    TestObserver testObserver;

    Glowdome3d gd3d;

    boolean useKinect;
    boolean useLeap;

    KinectTracker tracker;

    LeapMotion leap;

    PApplet thisApplet;

    PImage kinectImage;         // kinect is drawn into this
    PImage backgroundImage;
    PImage sourceImage;

    PGraphics offscreenBuffer;

    PImage currentImage;
    boolean autoCycle = true;

    File [] imageFiles;
    int currentImageNum;

    int renderMode = 0;
    int numModes = 7;
    boolean [] layerStatus;

    boolean textEntry = false;
    String textString = "HELLO, WORLD";

    PFont font20;
    PFont font80;

    float xCycle = 0;
    float yCycle = 0;

    float xSpeed = 1;
    float ySpeed = 0;

    int lastMillis = millis();
    int cycleMillis = millis();  // the last time the image was cycled
    int handsMillis = millis();
    int curMillis = millis();

    float imageTrace = 0;  // which column of pixels we are currently sending to the strips
    float traceSpeed = 1;  // how many pixels to skip each frame, adjust for motor speed

    int stripeWidth = 10;
    int slideshowDelay = 15000;

    int saturationAdjust = 0;
    int hueShift;

    boolean loadedMovie = false;
    Movie mov;

    // kinect variables
    int kw = 640;
    int kh = 480;
    int threshold = 1150;

    // point cloud
    int[] depth;

    float[] depthLookUp = new float[2048];

    float a = 0;
    
    PVector prevAverage;
    PVector handsDelta;
 
   int columnsPerSecond = 1;
    
    GlowdomeRender(PApplet applet, boolean kinect, boolean leap) {
        useKinect = kinect;
        useLeap = leap;
        thisApplet = applet;
        layerStatus = new boolean[10];
        
        handsDelta = new PVector(0, 0);
        prevAverage = new PVector(0, 0);
               
        // turn all layers off
        for (boolean currentStatus : layerStatus) {
            currentStatus = false;
        }
        
        layerStatus[1] = true;
    }

    public void setup() {
        colorMode(RGB, 255);

        registry = new DeviceRegistry();
        testObserver = new TestObserver();
        registry.addObserver(testObserver);
        
        /*
         However, if you\u2019re on a very slow network, like a very long wireless link or a cellular modem,
         you might want to add an extra slowdown.  That\u2019s what the registry.setExtraDelay() method is for.
        */
        //registry.setExtraDelay(1000);
        /*
        If you\u2019re using video source material and everything looks washed out, you may want to apply the antilog curve correction.  
        */
        registry.setAntiLog(true);
        
        
        /*
        If you\u2019re on a network with a high error rate, like some wireless networks, or poorly installed ethernet, 
        you might find that you get persistently high error rates and the update frequency drops uncontrollably.
        In this case, you may want to disable the autothrottling entirely.         
        */
        registry.setAutoThrottle(false);

        registry.startPushing();


        backgroundImage = createImage(width, height, RGB);
        kinectImage = createImage(kw, kh, RGB);
        sourceImage = loadImage("had001.png");

        offscreenBuffer = createGraphics(width, height, JAVA2D);

        if (useLeap) {
            leap = new LeapMotion(thisApplet).withGestures();
        }

        if (useKinect) {
            kinect = new Kinect(thisApplet);
            tracker = new KinectTracker();
            kinect.enableDepth(true);
            // We don't need the grayscale image in this example
            // so this makes it more efficient
            kinect.processDepthImage(false);

            // Lookup table for all possible depth values (0 - 2047)
            for (int i = 0; i < depthLookUp.length; i++) {
                depthLookUp[i] = rawDepthToMeters(i);
            }
        }
        
        setupFonts();
        
        gd3d = new Glowdome3d();
        gd3d.setup(thisApplet);
        readImagesFromDir();
    }
    
    public void setupFonts() {
        font20 = loadFont("fonts/SourceCodePro-Regular-20.vlw");
        font80 = loadFont("fonts/SourceCodePro-Regular-80.vlw");
    }

    public void stop() {
        if (useKinect) {
            tracker.quit();
        }
    }

    /**
     *   Create an array of image file names
     */
    private void readImagesFromDir() {
        String path = sketchPath+"/data/"; 
        imageFiles = listFiles(path);
    }

    public File[] listFiles(String dir) {
     File file = new File(dir);
     if (file.isDirectory()) {
       File[] files = file.listFiles();
       return files;
     } else {
       // If it's not a directory
       return null;
     }
    }

    public void cycleImage(Integer dir) {
        cycleMillis = curMillis;
        currentImageNum+=dir;

        if (currentImageNum > imageFiles.length - 1) {
            currentImageNum = 0;
        }
        
        if (currentImageNum < 0) {
            currentImageNum = imageFiles.length - 1;
        }
        
        println(imageFiles[currentImageNum]);
        
        if(!imageFiles[currentImageNum].isDirectory() 
          && imageFiles[currentImageNum].exists()
          && imageFiles[currentImageNum].getName().matches("(?i).*(jpg|png|jpeg)$")
        ) {
            sourceImage = loadImage(imageFiles[currentImageNum].getAbsolutePath());            
        } else { 
            cycleImage(2 * dir);
        }        
    }

    public void loadMovie(PApplet sketch) {
        mov = new Movie(sketch, "transit.mov");
        mov.loop();
    }

    /**
     * The main render method draws all the layers and console
     */
    public void render() {

        PVector kinectVector;
        PVector [] leapVectors;

        if (useKinect) {
            kinectVector = tracker.getPos();
        } else {
            kinectVector = new PVector(0, 0);
        }

        int handHum = 0;
        int numHands = 0;
        leapVectors = new PVector[2];
        
        if (useLeap) {
            for (Hand hand : leap.getHands()) {
                leapVectors[handHum++] = hand.getStabilizedPosition();
                numHands++;
            }
            
            // if no hands are present, set a default speed
            if (numHands == 0) {
               leapVectors[0] = new PVector(0, 0);
               leapVectors[1] = new PVector(0, 0);
               xSpeed = 1;
               ySpeed = 0;
            }
            updateDeltaVector(leapVectors);
        } else {
          leapVectors[0] = new PVector(0,0);
        }

        if ((handsDelta.x > 0 || handsDelta.y > 0) && handsMillis - millis() < 2000) {
          xSpeed = -1 * handsDelta.x;
          ySpeed = handsDelta.y;
          handsMillis = millis();
        }

        background(0);

        int currentLayer;
        
        if (useKinect) {
          tracker.track();
        }

        for (currentLayer=0; currentLayer <= numModes; currentLayer++) {
            if (layerStatus[currentLayer] == true) {
                switch(currentLayer) {
                    case 1:
                        renderPicture(leapVectors);
                        break;
                    case 2:
                        renderTest(kinectVector);
                        break;
                    case 3:
                        renderNoise(kinectVector, leapVectors);
                        break;
                    case 4:
                        renderPointCloud();
                        break;
                    case 5:
                        renderSphere();
                        break;
                    case 6:
                        renderText();
                        //renderKinect();
                        //tracker.display();
                        break;
                    case 7:
                        renderRings();
                        break;
                }
            }
        } 

        renderConsole();

        //image(offscreenBuffer, 0, 0);
    }

    /**
     *   Draw an image, handle shifting in x,y directions
     */
    public void renderPicture(PVector [] hands) {

        
        float xScale, yScale;
        float xPix, yPix;
        float xSrc, ySrc;

        int red;
        int green;

        int pixel;

        float pi = 3.14159f;

        int srcRowOffset;    // offset in pixels to the current row of the source image
        int destRowOffset;

        boolean xSquare, ySquare;

        curMillis = millis();
        
        if (autoCycle && curMillis - cycleMillis > slideshowDelay) {
          readImagesFromDir();         
          cycleImage(1);
        }

        colorMode(HSB, 255);

        offscreenBuffer.loadPixels();

        for (int y = 0; y < backgroundImage.height; y++) {
            float yOffsetted = (y + yCycle) % backgroundImage.height;
            yScale = (float)yOffsetted / backgroundImage.height;
            //yPix = sin(yScale * pi + pi/2);
            yPix = yScale;
            ySrc = (int)map(yPix, 0, 1, 0, sourceImage.height - 1);
            srcRowOffset = (int)ySrc * sourceImage.width;
            destRowOffset = y * backgroundImage.width;
            for (int x = 0; x < backgroundImage.width; x++) {
                float xOffsetted = (x + xCycle) % backgroundImage.width;

                xScale = (float)xOffsetted / backgroundImage.width;

                xSrc = (int)map(xScale, 0, 1, 0, sourceImage.width - 1);

                int offset = (int)(srcRowOffset + xSrc);

                pixel = sourceImage.pixels[offset];

                if (saturationAdjust != 0) {
                    float saturation = saturation(pixel);
                    saturation += saturationAdjust;
                    if (saturation > 255) saturation = 255;

                    pixel = color(hue(pixel), saturation, brightness(pixel));
                }

                offscreenBuffer.pixels[destRowOffset + x] = pixel;

            }
        }

        offscreenBuffer.updatePixels();

        //image(backgroundImage, 0, 0, width, height);
        blend(offscreenBuffer, 0, 0, width, height, 0, 0, width, height, LIGHTEST);

        colorMode(RGB, 255);
    }

    /**
     * Render a test pattern
     */
    public void renderTest(PVector v1) {
        offscreenBuffer.loadPixels();
        int prevX = 0;
        int inX;
        int stripeNum = 0;
        for (int y = 0; y < backgroundImage.height; y++) {

            for (int x = 0; x < backgroundImage.width; x++) {
                float xOffsetted = (x + xCycle) % backgroundImage.width;

                int offset = 0;

                int stripe = (int)(y + xCycle/2);
                int stripeX = (int)(x + yCycle/2);



                 inX = stripeX % stripeWidth < stripeWidth/2 ? 1 : 0;
                int inY = stripe % stripeWidth < stripeWidth/2 ? 1 : 0;

                if (prevX != inY) {
                    stripeNum++;
                }
                prevX = inX;

                int red = stripeNum % 2 == 1 ? 255 : 0;
                int green = inX == 1 && inY == 1 ? 255 : 0;
                int blue = inX == 0 && inY == 0 ? 255 : 0;

                offscreenBuffer.pixels[y * backgroundImage.width + x] = color(red, green, blue);
            }
        }
        offscreenBuffer.updatePixels();
        xCycle += xSpeed;

        //image(backgroundImage, 0, 0);
        blend(offscreenBuffer, 0, 0, width, height, 0, 0, width, height, LIGHTEST);
    }

    public void renderMovie() {
        image(mov, 0, 0);
    }

    public void renderRects() {
        randomSeed(0);
        fill(0, 255, 0);
        int rectHeight;
        for (int i=0; i < width; i++) {
            rectHeight = (int)random(0, height);
            rect(i, 0, 1, rectHeight);
        }
    }

    public void renderRings() {
        int lineSpacing = 50;

        for (int lineNum=0; lineNum < 20; lineNum++) {
            int r = 255 - (lineNum * 10);
            int b = (lineNum * 10) % 255;
            stroke(r, 0, b);
            strokeWeight(10);
            line(lineNum*lineSpacing,
                    lineNum*lineSpacing/2,
                    width - lineNum*lineSpacing/2,
                    height - lineNum*lineSpacing);
        }

        fill(0, 255, 0);
        strokeWeight(1);
        ellipse(50, 150, 50, 50);
    }

    public void renderText() {
        fill(255, 0, 0);
        textFont(font80);
        textSize(60);
        text(textString, 10, height/2);
    }

    public void renderConsole() {
        textFont(font20);
        textSize(20);
        text(traceSpeed + " position: " + xCycle + "\n" + imageFiles[currentImageNum].getName(), 20, height - 30);
        fill(255, 0, 0);
    }

    /**
     * Render kinect
     */
    public void renderKinect() {
        if (!useKinect) return;

        PImage img = kinect.getVideoImage();

        kinectImage.loadPixels();

        float avgDepth = 0;

        for(int x = 0; x < kw; x++) {
            for(int y = 0; y < kh; y++) {
                // mirroring image
                int offset = kw-x-1+y*kw;
                // Raw depth
                int rawDepth = tracker.depth[offset];

                avgDepth += rawDepth;

                int pix = x+y*kinectImage.width;
                if (rawDepth < threshold) {
                    //colourful twin
                    colorMode (HSB);
                    kinectImage.pixels[pix] = color(rawDepth % 360, 250, 150);
                    //println (rawDepth);
                }
                else {
                    //creating the mirrored world
                    colorMode (RGB);
                    float r = red (img.pixels[pix]);
                    float g = green (img.pixels[pix]);
                    float b = blue (img.pixels[pix]);

                    int c = color (r, g, b);
                    kinectImage.pixels[pix] = c;
                }
            }
        }
        kinectImage.updatePixels();

        avgDepth /= kw * kh;

        int depthPixel = (int)map(avgDepth, 0, 2000, 0, 255);

        fill(depthPixel);

        // Draw the image
        //image(kinectImage,0,0, width, height);
        blend(kinectImage, 0, 0, 640, 480, 0, 0, width, height, LIGHTEST);
        colorMode(RGB, 255);

    }

    public void renderPointCloud() {

        if (!useKinect) return;
      
        colorMode(HSB);
        strokeWeight(1);
        //textMode(SCREEN);
        //text("Kinect FR: " + (int)kinect.getDepthFPS() + "\nProcessing FR: " + (int)frameRate,10,16);

        // Get the raw depth as array of integers
        int[] depth = kinect.getRawDepth();

        // We're just going to calculate and draw every 4th pixel (equivalent of 160x120)
        int skip = 3;

        // Translate and rotate
        translate(width/2, height/2, -50);
        rotateY(a);

        for(int x=0; x< kw; x+=skip) {
            for(int y=0; y< kh; y+=skip) {
                int offset = x+y * kw;

                // Convert kinect data to world xyz coordinate
                int rawDepth = depth[offset];
                PVector v = depthToWorld(x,y,rawDepth);

                //print(rawDepth + " ");

                int hue = (int)map(rawDepth, 500, 2100, 0, 255);

                stroke(200, 200, 220);
                pushMatrix();
                // Scale up by 200
                float factor = 200;
                translate(v.x*factor,v.y*factor,factor-v.z*factor);
                // Draw a point
                point(0,0);
                popMatrix();
            }
        }

        // Rotate
        a += 0.015f;
    }

    /**
     * Render perlin noise and shift, change color using kinect and leap
     * @param PVector v1
     * @param PVector [] fingers
     */
    public void renderNoise(PVector v1, PVector [] fingers) {
        float noiseScale = .10f;

        int fingerCount = 0;
        float fingerAvg = 0;
        for(PVector finger : fingers) {
            if (finger == null) break;
            fingerCount++;
            fingerAvg += finger.x;
        }
        fingerAvg /= fingerCount;

        int red = (int)map(fingerAvg, 0, 500, 0, 255);

        for (int y = 0; y < backgroundImage.height; y++) {

            int destRowOffset = y * backgroundImage.width;
            for (int x = 0; x < backgroundImage.width; x++) {
                float xOffsetted = (x + xCycle) % backgroundImage.width;

                //xScale = (float)xOffsetted / backgroundImage.width;

                //xPix = (xScale);

                int noiseR = (int)(noise((float)(x + v1.x) * noiseScale, (float)(y + v1.y) * noiseScale) * 250);
                //int noiseG = (int)(noise((float)(x + xCycle) * noiseScale * 2, (float)y * noiseScale) * 250);
                //int noiseB = (int)(noise((float)(x + xCycle) * noiseScale * 2, (float)(y + xCycle) * noiseScale) * 250);

                int pixel = color(noiseR, red, 0);

                backgroundImage.pixels[destRowOffset + x] = pixel;

            }
        }

        backgroundImage.updatePixels();
        
        blend(backgroundImage, 0, 0, width, height, 0, 0, width, height, LIGHTEST);
        //image(backgroundImage, 0, 0, width, height);
    }

    /**
        Render a sphere, control with leap
     */
    public void renderSphere() {

      gd3d.render(handsDelta);

    }

    /**
        Read the current column of pixels from the final image, send to pixelpusher
     */
    public void display() {
        int c;
                
        if (testObserver.hasStrips) {
            int stripNum = 0;
            List<Strip> strips = registry.getStrips();

            if (strips.size() > 0) {
                int yscale = height / strips.size();
                
                
//                for(Strip strip : strips) {
                Strip strip = strips.get(0);
                //if(numStripsOverride == 1) {
                //    strip = strips.get(0);
                //}


                  int stripLength = strip.getLength();
                    int xscale = width / stripLength;
                    yscale = height / stripLength;
                    //println(stripNum);
                    
                    //LET's DUPE IMAGE ON XXXXXX
                    
                    for (int stripY = 0; stripY < stripLength; stripY++) {
                        c  = 0;
                        // interlace the pixel between the strips
                        
                        if (stripY % 2 == stripNum) {  // even led
                            c = get((int)imageTrace, stripY*yscale);
                            //c = offscreenBuffer.pixels[(int)imageTrace + stripY*yscale*width];
                            //c = color(255,0,0);
                        } else {    // odd led
                            if(interlaceColumns) {
                              c = get((int)imageTrace, stripY*yscale + 1);
                            } else {
                              c = get((int)imageTrace, stripY*yscale);                            
                            }
                            //c = offscreenBuffer.pixels[(int)imageTrace + ((int)(stripY*yscale) + 1)*width];
                            //println(stripY*yscale*width);
                        }
                        strip.setPixel(c, stripLength - stripY);
                    }
                    stripNum++;
                //}
            }
        }
                
        // SKIPS PIXELS
        imageTrace += traceSpeed;
        
        if (imageTrace > width - 1) imageTrace = imageTrace - width;

        // check for cycle going beyond the image


        //plug in leap coords
        xCycle += xSpeed;

        if (xCycle < 0) xCycle = width + xCycle;
        if (xCycle > width) xCycle = xCycle - width;

        //plug in leap coords          
        yCycle += ySpeed;
        //set mins maxes
        if (yCycle < 0) yCycle = height + yCycle;
        if (yCycle > height) yCycle = yCycle - height;
        
        if ((int)xCycle == 1) {
          println("ROTATION:" + (millis() / 1000) + " Elapsed: " + (millis() - last) + " RPM: " + (60.0f / (millis() - last)) * 1000.0f);
          // (60.0 / value) * 1000
                  last = millis();
        
        } else {
         // print("xCycle:" + xCycle);
         // println(" xSpeed:" + xSpeed);
        }
    }

    /**
     * Toggle a layer
     * @param int layerNum
     */
    public void toggleLayer(int layerNum) {
        layerStatus[layerNum] = !layerStatus[layerNum];
        println(layerNum);
    }

    /**
     * Clear layers
     */
    public void clearLayers() {
         for (int layerNum = 0; layerNum <= numModes; layerNum++) {
            layerStatus[layerNum] = false;
         } 
    }

    public void toggleTextEntry() {
        textEntry = !textEntry;
    }

    public void sendKey(char key) {
        textString += key;
    }

    public void resetSpeed() {
        xSpeed = 0;
        ySpeed = 0;
        xCycle = 0;
        yCycle = 0;
    }

    private void updateDeltaVector(PVector [] hands) {
      PVector average = averageVectors(hands);

        float xDelta, yDelta;

        if (!Double.isNaN(average.x)) {
            xDelta = average.x - prevAverage.x;
            yDelta = -(average.y - prevAverage.y);

            prevAverage.x = average.x;
            prevAverage.y = average.y;
        } else {
             xDelta = 0;
             yDelta = 0; 
        }
        handsDelta.x = xDelta;
        handsDelta.y = yDelta;
    }

    // These functions come from: http://graphics.stanford.edu/~mdfisher/Kinect.html
    public float rawDepthToMeters(int depthValue) {
        if (depthValue < 2047) {
            return (float)(1.0f / ((double)(depthValue) * -0.0030711016f + 3.3309495161f));
        }
        return 0.0f;
    }

    public PVector depthToWorld(int x, int y, int depthValue) {

        final double fx_d = 1.0f / 5.9421434211923247e+02f;
        final double fy_d = 1.0f / 5.9104053696870778e+02f;
        final double cx_d = 3.3930780975300314e+02f;
        final double cy_d = 2.4273913761751615e+02f;

        PVector result = new PVector();
        double depth =  depthLookUp[depthValue];//rawDepthToMeters(depthValue);
        result.x = (float)((x - cx_d) * depth * fx_d);
        result.y = (float)((y - cy_d) * depth * fy_d);
        result.z = (float)(depth);
        return result;
    }

    public PVector averageVectors(PVector [] vectors) {
        PVector average = new PVector();
        int fingerCount = 0;
        float fingerAvg = 0;
        for(PVector finger : vectors) {
            if (finger == null) break;
            fingerCount++;
            average.x += finger.x;
            average.y += finger.y;
        }
        
        if (fingerCount > 1) {
          average.x /= fingerCount;
          average.y /= fingerCount;
        }

        return average;
    }
}
  static public void main(String[] passedArgs) {
    String[] appletArgs = new String[] { "glowdome" };
    if (passedArgs != null) {
      PApplet.main(concat(appletArgs, passedArgs));
    } else {
      PApplet.main(appletArgs);
    }
  }
}
