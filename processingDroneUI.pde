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


boolean  ifDrawHumanObj = true;
boolean ifRosBridge = false;

boolean isEditing = false;
boolean isPreviewing = false;
int previewObjectIndex = 0;

//split screen
PGraphics skeletonModel;
PGraphics videoStream;

int skeletonModel_w = 640;
int skeletonModel_h = 800;
int videoStream_w = 640;
int videoStream_h = 400;
int map_w = 640;
int map_h = 400;

PShape human;

PeasyCam cam;
float globalPitch = 0;
float globalYaw = 0;

PIDController pid_globalPitch;
PIDController pid_globalYaw;


float drawingScale = 50;

int a = 1;

UnfoldingMap unfoldingMap;
ScreenPosition droneMapPosition;
Location droneLocation;
PImage plane;

PImage mapImage;
PImage videoFrame;
ArrayList<Float> dataArray = new ArrayList<Float>();

ArrayList<CamPose> selectedPoints = new ArrayList<CamPose>();



void setup() {

  // need to run: roslaunch rosbridge_server rosbridge_websocket.launch 

  size(1280, 800, OPENGL);


  skeletonModel = createGraphics(skeletonModel_w, skeletonModel_h, OPENGL);
  videoStream = createGraphics(videoStream_w, videoStream_h, P2D);

  mapImage = loadImage("map.png");

  cam = new PeasyCam(this, skeletonModel, 400);
  cam.setMinimumDistance(200);
  cam.setMaximumDistance(3000);
  cam.setLeftDragHandler(new PeasyDragHandler() {
    public void handleDrag(final double dx, final double dy) {
      if (globalPitch+dy*0.01>=0&&globalPitch+dy*0.01<=PI/2.0) {
        globalPitch+=dy*0.008;
      }
      globalYaw+=dx*0.008;
    }
  }
  );

  pid_globalPitch = new PIDController(0.1, 0.2, 0);
  pid_globalYaw = new PIDController(0.1, 0.2, 0);



  int trajectoryPlanningSampleRate = 100; //Hz

  human = loadShape("Mii.obj");  

  Publisher p;

  PrimitiveMsg pp;


  if (ifRosBridge) {

    String URI = "ws://localhost:9090";

    RosBridge bridge = new RosBridge();
    bridge.connect(URI, true);

    bridge.subscribe(SubscriptionRequestMsg.generate("/xyzr")
      .setType("std_msgs/Float32MultiArray")
      .setFragmentSize(65000)
      .setThrottleRate(1)
      .setQueueLength(1), 
    new RosListenDelegate() {
      public void receive(JsonNode data, String stringRep) {
        ArrayNode slaidsNode = (ArrayNode)  data.get("msg").get("data");
        Iterator<JsonNode> slaidsIterator = slaidsNode.elements();
        //dataArray.clear();
        while (slaidsIterator.hasNext ()) {
          JsonNode slaidNode = slaidsIterator.next();
          dataArray.add(slaidNode.floatValue());
        }
        System.out.println("dataArray.size() = "+dataArray.size());
      }
    }
    );

    bridge.subscribe(SubscriptionRequestMsg.generate("/webcam/image_raw")
      .setType("sensor_msgs/Image")
      .setFragmentSize(50000)
      .setThrottleRate(1)
      .setQueueLength(1), 
    new RosListenDelegate() {

      public void receive(JsonNode data, String stringRep) {

        int h = data.get("msg").get("height").asInt();
        int w = data.get("msg").get("width").asInt();
        String encoding = data.get("msg").get("encoding").textValue();

        ObjectMapper om = new ObjectMapper();
        final ObjectWriter writer = om.writer();

        // Use the writer for thread safe access.
        try {
          //byte[] imageData = Base64.getDecoder().decode(data.get("msg").get("data").textValue());
          //ToPImage tpi = new ToPImage(h, w, encoding, imageData);
          //videoFrame = tpi.getPImage();
        }

        catch(Exception e) {
          println(e.toString());
        }
      }
    }
    );
  }



  unfoldingMap = new UnfoldingMap(this, skeletonModel_w, videoStream_h, map_w, map_h, new OpenStreetMap.OpenStreetMapProvider());
  unfoldingMap.zoomAndPanTo(new Location(52.5f, 13.4f), 10);
  MapUtils.createDefaultEventDispatcher(this, unfoldingMap);

  plane = loadImage("plane4.png");
}


void draw() {

  //controllers update
  if (pid_globalPitch.isOn()) {
    globalPitch = pid_globalPitch.process(globalPitch);
  }

  if (pid_globalYaw.isOn()) {
    globalYaw = pid_globalYaw.process(globalYaw);
  }

  //println(frameRate);
  //skeleton model
  skeletonModel.beginDraw();
  if (isEditing) {
    skeletonModel.background(backGourd_edit);
  } else {
    skeletonModel.background(backGourd_review);
  }

  skeletonModel.rotateX(PI-globalPitch);
  skeletonModel.rotateY(PI-globalYaw);

  skeletonModel.strokeWeight(1);
  skeletonModel.pushMatrix();
  rectGrid(25, (int)(1*drawingScale), 0);//a rectangle grid can be a lot bigger than a boxgrid, without caursing lag
  skeletonModel.popMatrix();




  if (ifDrawHumanObj) {
    skeletonModel.scale(drawingScale/200);
    skeletonModel.shape(human);
    skeletonModel.scale(200/drawingScale);
  } else {
    skeletonModel.stroke(0);
    skeletonModel.fill(255);
    skeletonModel.box(drawingScale);
  }
  skeletonModel.scale(drawingScale);
  skeletonModel.strokeWeight(1/drawingScale);
  //println("selectedPoints.size () = "+selectedPoints.size());

  for (int i = 0; i<selectedPoints.size (); i++) {
    skeletonModel.pushMatrix();
    float[] tempLoc = selectedPoints.get(i).getCartesian();
    skeletonModel.translate(tempLoc[0], tempLoc[1], tempLoc[2]);
    skeletonModel.stroke(0);

    double mouseObjectDistance = Math.sqrt(sq(mouseX-skeletonModel.screenX(0, 0, 0))+sq(mouseY-skeletonModel.screenY(0, 0, 0)));

    if (mouseObjectDistance<20) {
      skeletonModel.fill(255, 0, 0);
    } else {
      skeletonModel.fill(255);
    }

    skeletonModel.box(2);
    skeletonModel.popMatrix();


    skeletonModel.stroke(255);
    if (i!=0) {
      float[] tempLoc_last = selectedPoints.get(i-1).getCartesian();
      skeletonModel.line(tempLoc[0], tempLoc[1], tempLoc[2], tempLoc_last[0], tempLoc_last[1], tempLoc_last[2]);
    }
  }




  if (dataArray!=null&&dataArray.size()!=0) {
    for (int i = 4; i < dataArray.size (); i+=4) {
      skeletonModel.stroke(255);

      skeletonModel.line(dataArray.get(i-4), dataArray.get(i+2-4), dataArray.get(i+1-4), dataArray.get(i), dataArray.get(i+2), dataArray.get(i+1));
    }

    skeletonModel.noStroke();

    int index = ((frameCount*10)%dataArray.size())/4;
    if (index*4+2<dataArray.size()) {
      skeletonModel.pushMatrix();
      //println("index = "+index);
      //System.out.println(dataArray.get(index*4)+","+ dataArray.get(index*4+1)+","+ dataArray.get(index*4+2));

      skeletonModel.translate(dataArray.get(index*4), dataArray.get(index*4+2), dataArray.get(index*4+1));
      float yaw = dataArray.get(index+3);
      skeletonModel.rotateY(yaw);
      skeletonModel.fill(map(index*4, 0, dataArray.size(), 255, 50), 255, map(index*4, 0, dataArray.size(), 50, 255));
      skeletonModel.box(0.1);
      skeletonModel.popMatrix();
    }
  }

  skeletonModel.endDraw();





  //video stream
  videoStream.beginDraw();
  if (videoFrame!=null)
    videoStream.image(videoFrame, 0, 0, videoStream_w, videoStream_h);
  videoStream.endDraw();





  //map
  //map.background(100);
  unfoldingMap.draw();



  stroke(20, 20, 20, 100);
  fill(20, 20, 20, 100);
  if (droneLocation!=null) {
    droneMapPosition = unfoldingMap.getScreenPosition(droneLocation);
    //println(droneMapPosition);
    pushMatrix();
    translate(droneMapPosition.x, droneMapPosition.y);
    rotate(PlanePoseEmulator.getInstance().nextRotation());
    imageMode (CENTER); 
    image(plane, 0, 0);
    imageMode (CORNER); 
    popMatrix();
  }








  image(skeletonModel, 0, 0);
  image(videoStream, skeletonModel_w, 0);
  //image(map, skeletonModel_w, videoStream_h,map_w,map_h);
}

void rectGrid(int size, int tilesize, float y) {
  skeletonModel.noFill();//i only want the outline of the rectangles
  for (float x = -size/2; x <= size/2; x++) {
    for (float z = -size/2; z <= size/2; z++) {
      //run two for loops, cycling through 10 different positions of rectangles
      skeletonModel.pushMatrix();
      if (isEditing) {
        skeletonModel.stroke(groundMesh_edit, map(dist(0, 0, x*tilesize, z*tilesize), 0, size/2*tilesize, 255, 0));//the rectangles close to you, are clear, while the ones farther from you, are much fainter
      } else {
        skeletonModel.stroke(groundMesh_review, map(dist(0, 0, x*tilesize, z*tilesize), 0, size/2*tilesize, 255, 0));//the rectangles close to you, are clear, while the ones farther from you, are much fainter
      }
      //uncomment the next line:
      //stroke(0,255,0);
      // to see how the infinity thing works

      skeletonModel.translate(x*tilesize, y, z*tilesize);//move the rectangles to where they shall be
      skeletonModel.rotateX(HALF_PI);
      skeletonModel.rect(0, 0, tilesize, tilesize);
      skeletonModel.popMatrix();
    }
  }
}


void keyPressed() {
  if (key=='r') {
    if (isEditing) {
      println("cam.getDistance = "+cam.getDistance());
      println("globalPitch = "+globalPitch);
      println("globalYaw = "+globalYaw);
      float R = (float)cam.getDistance()/drawingScale;
      float newZ = R*cos(globalYaw)*cos(globalPitch);
      float newY = R*sin(globalPitch);
      float newX = R*cos(globalPitch)*sin(globalYaw);
      selectedPoints.add(new CamPose(R, globalPitch, globalYaw));
    }
  }


  if (key == 'e') {
    //switch between reveiw and edit mode
    isEditing = !isEditing;
    println("isEditing = "+isEditing);
  }

  if (key == 'l') {
    //locate, refresh for current GPS, update map
    unfoldingMap.zoomAndPanTo(new Location(34.4131f, -119.845f), 10);
    droneLocation = new Location(34.4184916f, -119.8566171f);

    println("droneMapPosition = "+droneMapPosition);
  }

  if (key == 'h') {
    ifDrawHumanObj = !ifDrawHumanObj;
    println("ifDrawHumanObj = "+ifDrawHumanObj);
  }

  if (key == 'h') {
    isPreviewing = !isPreviewing;
    println("isPreviewing = "+isPreviewing);
  }



  if (key=='1') {
    //go to front view

    //    globalPitch = 0.2108;
    //    globalYaw = 0;
    pid_globalPitch.setReference(0.2108f);
    pid_globalYaw.setReference(0);
  }

  if (key=='2') {
    //go to side view

    //    globalPitch = 0.2108;
    //    globalYaw = PI/2.0f;
    pid_globalPitch.setReference(0.2108f);
    pid_globalYaw.setReference(PI/2.0f);
  }


  if (key=='3') {
    //go to top view

    //    globalPitch = PI/2.0f;
    //    globalYaw = 0;
    pid_globalPitch.setReference(PI/2.0f);
    pid_globalYaw.setReference(0);
  }
}

