using System.Collections;
using System.Collections.Generic;
//using System.Collections.IEnumerable;
using UnityEngine;
using System;
using LCM;

public class heightmap_mesh_update_LCM : MonoBehaviour
{
  //Mesh Renderer variables
  Mesh mesh;
  Vector3[] vertices; //array of positions in column-major form
  private static int[] indices; //array of mesh lines indices
  Color[] mesh_colors;
  public Gradient gradient;

  //LCM Grid Map Message variables
  private static LCMTypes.grid_map_lcmt msg;
  private static int rows = 0;
  private static int cols = 0;
  private static Vector3 center = Vector3.zero;
  private static Vector3 last_center= center; //new Vector3(-1,-1,-1); //set a position that is not (0,0,0)
  private static float resolution = 0.01f;

  private static float[] heightmap;
  private static int[] indexmap;

  private static int total_vertices=rows*cols;
  private static int last_total_vertices=rows*cols;
  private static ulong iter = 0;
  bool first_msg=true;

  //Hardcoded parameters for colors
  Color bottom_color= Color.blue;
  Color mid_color= Color.green;
  Color top_color= Color.red;
  float min_height=0; //shoulldn't be hard coded
  float max_height=1;

  // Start is called before the first frame update
  void Start()
  {
    StartCoroutine("Listener");
    StartMesh();
  }

  // Update is called once per frame
  void Update()
  {
    iter = iter + 1;
    if(iter%3 ==0){
      UpdateMesh();
    }
  }

  private void StartMesh(){ //Initialize mesh renderer
    //Create mesh and set colors and materials
    mesh=  new Mesh();
    GetComponent<MeshFilter>().mesh=mesh;
    mesh.Clear();
    gradient.SetKeys(
        new GradientColorKey[] { new GradientColorKey(bottom_color, 0.0f),new GradientColorKey(mid_color, 0.5f), new GradientColorKey(top_color, 1.0f) },
        new GradientAlphaKey[] { new GradientAlphaKey(1.0f, 0.0f), new GradientAlphaKey(1.0f, 1.0f) }
        );
    gameObject.GetComponent<MeshRenderer>().material=new Material(Shader.Find("Sprites/Default"));
  }    

  private void UpdateMesh(){ //Update mesh if height map information has changed      

    //if(total_vertices !=0 && last_center !=center){ //if receiving a nonzero height map and robot has moved. Maybe should change to distance is within some threshold
    if(total_vertices !=0 ){ //if receiving a nonzero height map and robot has moved. Maybe should change to distance is within some threshold
      // var startTime= DateTime.Now;
      mesh.Clear(); //reset last msg

      //Resize arrays if necessary
      if (last_total_vertices!=total_vertices || first_msg){
        Array.Resize<Vector3>(ref vertices,total_vertices);
        Array.Resize<int>(ref indices, 2*(2*rows*cols-(rows+cols)));
        Array.Resize<Color>(ref mesh_colors, total_vertices);
        first_msg=false;          
      }

      //Convert heightmap to vertices (Vec3 1D array of positions in column major form) and assign colors
      float x_start=resolution*Mathf.Floor(rows/2)+center[0];
      float y_start=resolution*Mathf.Floor(cols/2)+center[2];
      for (int c=0,vert=0;c<cols;c++){ //change int for other bigger type
        for (int r=0; r<rows; r++){
          vertices[vert]=new Vector3(x_start-r*resolution,(float)heightmap[vert],y_start-c*resolution);
          //Set the color of the vertex based on the defined gradient
          // float norm_height=Mathf.InverseLerp(min_height,max_height,vertices[vert].y);
          // mesh_colors[vert]=gradient.Evaluate(norm_height);
          if(indexmap[vert]>=1){
            mesh_colors[vert]=Color.blue;
          }
          else{
            mesh_colors[vert]=Color.green;
          }
          vert++;
        }
      }

      //Create row lines
      int i=0;
      for (int r=0;r<rows;r++){ //change int for other bigger type
        for (int c=0; c<cols-1; c++){
          indices[i]=rows*c+r; //COLUMN MAJOR FORM
          indices[i+1]=rows*(c+1)+r;
          i+=2;
        }
      }


      //Create column lines
      for (int c=0; c<cols; c++){ //change int for other bigger type
        for (int r=0;r<rows-1;r++){
          indices[i]=rows*c+r;
          indices[i+1]=rows*(c)+(r+1);
          i+=2;
        }
      }

      //Set mesh parameters
      mesh.vertices = vertices;
      mesh.SetIndices(indices, MeshTopology.Lines, 0, false);
      mesh.colors=mesh_colors;

      // Debug.Log("Update Time: "+(DateTime.Now - startTime).Milliseconds);
      last_total_vertices=total_vertices; //keeps track if height map changes size
      last_center=center;
    }
  }

  internal class SimpleSubscriber : LCM.LCM.LCMSubscriber
    {

      public void MessageReceived(LCM.LCM.LCM lcm, string channel, LCM.LCM.LCMDataInputStream dins)
      {
        // Debug.Log ("RECV: " + channel);
        if (channel == "HEIGHTMAP"){
          msg = new LCMTypes.grid_map_lcmt(dins);

          rows=msg.rows;
          cols=msg.cols;
          center=new Vector3((float)msg.center[0],0.0f,(float)msg.center[1]);
          resolution=(float)msg.resolution;
          if(total_vertices!=msg.totalCells){ // Resize if size of height map has changed
            total_vertices=msg.totalCells;
            Array.Resize<float>(ref heightmap, total_vertices);
            Array.Resize<int>(ref indexmap, total_vertices);
          }
          heightmap=msg.heightmap;
          indexmap=msg.indexmap;
      }
    }
  }

  Boolean running = false;
  LCM.LCM.LCM myLCM = null;

  public IEnumerator Listener()
  {
    Debug.Log ("Debug listener started.");
    myLCM = new LCM.LCM.LCM();

    myLCM.Subscribe("HEIGHTMAP", new SimpleSubscriber());
    // may want to subscribe only to one channel: myLCM.Subscribe("quadruped_visualization_info", new SimpleSubscriber());
    running = true;
    while (running){
      yield return null;
    }
    Debug.Log ("Debug listener coroutine returning!");
  }
}
