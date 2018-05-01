class CamPose {
  String title;
  float[] cartesianCoord;
  float[] sphericalCoord;

  //contructor with Cartesian Coordinates input
  public CamPose(float R, float pitch, float yaw) {

    float newZ = R*cos(yaw)*cos(pitch);
    float newY = R*sin(pitch);
    float newX = R*cos(pitch)*sin(yaw);

    cartesianCoord = new float[] {
      newX, newY, newZ
    };
    sphericalCoord = new float[] {
      R, pitch, yaw
    };
    
    title = "Untitled";
  }
  
    public CamPose(float R, float pitch, float yaw,String  title) {

    float newZ = R*cos(yaw)*cos(pitch);
    float newY = R*sin(pitch);
    float newX = R*cos(pitch)*sin(yaw);

    cartesianCoord = new float[] {
      newX, newY, newZ
    };
    sphericalCoord = new float[] {
      R, pitch, yaw
    };
    
    this.title = title;
  }
  
  
  
//  //contructor with Spherical Coordinates input
//  public CamPose(float R, float pitch, float yaw) {
//    //to be continue...
//  }
  
  public float getSphericalR(){
    return  sphericalCoord[0];
  }
  
  public float getSphericalPitch(){
    return  sphericalCoord[1];
  }
  
  public float getSphericalYaw(){
    return  sphericalCoord[2];
  }
  
  public float[] getSpherical(){
    return  sphericalCoord;
  }
  
  
  
  public float getCartesianX(){
    return  cartesianCoord[0];
  }
  
  public float getCartesianY(){
    return  cartesianCoord[1];
  }
  
  public float getCartesianZ(){
    return  cartesianCoord[2];
  }
  
  public float[] getCartesian(){
    return  cartesianCoord;
  }
  
  public String getTitle(){
   return title; 
  }
  
}

