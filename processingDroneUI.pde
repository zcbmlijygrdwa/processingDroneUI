import peasy.*;
import peasy.org.apache.commons.math.*;
import peasy.org.apache.commons.math.geometry.*;

import java.io.*;

import java.util.Iterator; 

import javax.imageio.ImageIO;

//import java.util.Base64;
import ros.Publisher;
import ros.RosBridge;
import ros.RosListenDelegate;
import ros.SubscriptionRequestMsg;
import ros.msgs.std_msgs.PrimitiveMsg;
import ros.tools.MessageUnpacker;

import com.fasterxml.jackson.databind.JsonNode;

import java.awt.image.BufferedImage;



/**
 * imports for unfolding map
 * An application with a basic interactive map. You can zoom and pan the map.
 */

import de.fhpotsdam.unfolding.*;
import de.fhpotsdam.unfolding.geo.*;
import de.fhpotsdam.unfolding.utils.*;
import de.fhpotsdam.unfolding.providers.*;


PVector gridPos = new PVector(0, 0, 0);//the position of the grid (it moves with the camera to make it look infinite)

PeasyCam cam;
float globalPtich = 0;
float globalYaw = 0;

float drawingScale = 50;

int a = 1;

PImage img;
ArrayList<Float> dataArray = new ArrayList<Float>();
void setup() {

  // need to run: roslaunch rosbridge_server rosbridge_websocket.launch 

  size(500, 500, OPENGL);

  cam = new PeasyCam(this, 800);
  cam.setMinimumDistance(360);
  cam.setLeftDragHandler(new PeasyDragHandler() {
    public void handleDrag(final double dx, final double dy) {
      if (globalPtich+dy*0.01>=0&&globalPtich+dy*0.01<=PI/2.0) {
        globalPtich+=dy*0.008;
      }
      globalYaw+=dx*0.008;
    }
  }
  );


  int trajectoryPlanningSampleRate = 100; //Hz

  println(a); 

  Publisher p;

  PrimitiveMsg pp;


  String URI = "ws://localhost:9090";

  RosBridge bridge = new RosBridge();
  bridge.connect(URI, true);

  bridge.subscribe(SubscriptionRequestMsg.generate("/xyzr")
    .setType("std_msgs/Float32MultiArray")
    .setFragmentSize(50000)
    .setThrottleRate(1)
    .setQueueLength(1), 
  new RosListenDelegate() {

    public void receive(JsonNode data, String stringRep) {
      ArrayNode slaidsNode = (ArrayNode)  data.get("msg").get("data");
      Iterator<JsonNode> slaidsIterator = slaidsNode.elements();
      dataArray.clear();
      while (slaidsIterator.hasNext ()) {
        JsonNode slaidNode = slaidsIterator.next();
        dataArray.add(slaidNode.floatValue());
      }
      System.out.println("dataArray.size() = "+dataArray.size());
    }
  }
  );



  //run the second window
  runSketch( new String[] { 
    "--display=1", 
    "--location=0,0", 
    "--sketch-path=" + sketchPath, 
    ""
  }
  , 
  new geoMapSketch() );
}


void draw() {
  background(0);

  rotateX(PI-globalPtich);
  rotateY(PI-globalYaw);


  strokeWeight(1);
  pushMatrix();
  translate(gridPos.x, gridPos.y, gridPos.z);//everything that apears to be folowing the the center of the grid, goes after here
  rectGrid(25, (int)(1*drawingScale), 0);//a rectangle grid can be a lot bigger than a boxgrid, without caursing lag


  popMatrix();

  scale(drawingScale);


  stroke(0);
  strokeWeight(1/drawingScale);
  fill(255);
  box(1);



  if (dataArray!=null&&dataArray.size()!=0) {
    for (int i = 4; i < dataArray.size (); i+=4) {
      stroke(255);
      line(dataArray.get(i-4), dataArray.get(i+1-4), dataArray.get(i+2-4), dataArray.get(i), dataArray.get(i+1), dataArray.get(i+2));
    }

    noStroke();
    pushMatrix();
    int index = ((frameCount*10)%dataArray.size())/4;
    println("index = "+index);
    translate(dataArray.get(index*4), dataArray.get(index*4+1), dataArray.get(index*4+2));
    float yaw = dataArray.get(index+3);
    rotateY(yaw);
    fill(map(index*4, 0, dataArray.size(), 255, 50), 255, map(index*4, 0, dataArray.size(), 50, 255));
    box(0.1);
    popMatrix();
  }
}

void rectGrid(int size, int tilesize, float y) {
  noFill();//i only want the outline of the rectangles
  for (float x = -size/2; x <= size/2; x++) {
    for (float z = -size/2; z <= size/2; z++) {
      //run two for loops, cycling through 10 different positions of rectangles
      pushMatrix();

      stroke(0, 255, 0, map(dist(-gridPos.x, -gridPos.z, x*tilesize, z*tilesize), 0, size/2*tilesize, 255, 0));//the rectangles close to you, are clear, while the ones farther from you, are much fainter
      //uncomment the next line:
      //stroke(0,255,0);
      // to see how the infinity thing works

      translate(x*tilesize, y, z*tilesize);//move the rectangles to where they shall be
      rotateX(HALF_PI);
      rect(0, 0, tilesize, tilesize);
      popMatrix();
    }
  }
}


class geoMapSketch extends PApplet {
  UnfoldingMap map;

  PImage plane;
  PlanePoseEmulator instance;
  void setup() {
    println("Inner's sketchPath: \t\"" + sketchPath("") + "\"");
    println("Inner's dataPath: \t\"" + dataPath("") + "\"\n");

    size(500, 500);


    // providers:
    //OpenStreetMap.OpenStreetMapProvider()
    //OpenStreetMap.CloudmadeProvider(API KEY, STYLE ID)
    //StamenMapProvider.Toner()
    //Google.GoogleMapProvider()
    //Google.GoogleTerrainProvider()
    //Microsoft.RoadProvider()
    //Microsoft.AerialProvider()
    //Yahoo.RoadProvider()
    //Yahoo.HybridProvider()

    map = new UnfoldingMap(this, new OpenStreetMap.OpenStreetMapProvider());
    map.zoomAndPanTo(new Location(52.5f, 13.4f), 10);
    MapUtils.createDefaultEventDispatcher(this, map);

    plane = loadImage("plane4.png");
    instance = PlanePoseEmulator.getInstance();
  }

  void draw() {
    tint(255, 255);  // Display at full opacity
    background(100);
    map.draw();
    Location berlinLocation = new Location(52.5, 13.4);
    ScreenPosition posBerlin = map.getScreenPosition(berlinLocation);

    println(posBerlin);
    stroke(20, 20, 20, 100);
    fill(20, 20, 20, 100);

    pushMatrix();

    translate(posBerlin.x, posBerlin.y);
    //translate(mouseX,mouseY);

    //translate(-plane.width/2.0, -plane.height/2.0);
    rotate(0.1*frameCount);
    tint(255, 255*0.6);  // Display at 0.6 opacity
    imageMode (CENTER); 
    image(plane, 0, 0);
    imageMode (CORNER); 
    popMatrix();
  }
}

