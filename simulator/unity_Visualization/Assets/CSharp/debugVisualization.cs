using System.Collections;
using System.Collections.Generic;
//using System.Collections.IEnumerable;
using UnityEngine;
using System;
using LCM;

public class debugVisualization : MonoBehaviour
{
    private static int numLegs = 4;
    private static double[,] f_contact = new double[numLegs, 3]; // contact forces from LCM
    private static double[,] p_contact = new double[numLegs, 3]; // contact positions from LCM

    private float scaleFactor = 200;

    // unity red force arrow objects
    // could put this in array to clean up code in future, todo - sehwan
    GameObject force1;
    GameObject force2;
    GameObject force3;
    GameObject force4;

    // positions and forces for contact
    Vector3[] positions = { new Vector3 (0,0,0),
                            new Vector3 (0,0,0),
                            new Vector3 (0,0,0),
                            new Vector3 (0,0,0)};

    Vector3[] forces = { new Vector3 (0,0,0),
                         new Vector3 (0,0,0),
                         new Vector3 (0,0,0),
                         new Vector3 (0,0,0)};

    Vector3 unityPositionReshape = new Vector3(0,0,0);
    Vector3 unityForceReshape = new Vector3(0,0,0);

    Vector3 scale1 = new Vector3 (0,0,0);
    Vector3 scale2 = new Vector3 (0,0,0);
    Vector3 scale3 = new Vector3 (0,0,0);
    Vector3 scale4 = new Vector3 (0,0,0);

    /* Sphere Variables & Parameters */
    private static int sphere_count;
    private static int last_sphere_count;
    private static int max_spheres=100;
    Vector3 default_sphere_scale = new Vector3(0.1f,0.1f,0.1f);
    Vector3 default_sphere_position = new Vector3(0,-2,0);
    Color default_sphere_color = new Color(0.0f,0.0f,1.0f,.50f);
    private GameObject sphere;
    public List<GameObject> spheres = new List<GameObject>();
    private static Renderer[] sphere_renderers= new Renderer[max_spheres]; //caching the renderers so it doesn't have to getComponent on update (not sure if necessary)
    private static Vector3[] sphere_positions= new Vector3[max_spheres];
    private static Vector3[] sphere_scales= new Vector3[max_spheres];
    private static Color[] sphere_colors= new Color[max_spheres];


    /* Path Variables & Parameters */
    private static int path_count;
    private static int last_path_count;
    private static int max_paths=50;
    private static int max_path_points=200;
    Color default_path_color = new Color(1.0f,1.0f,0.0f,.50f);
    private GameObject path;
    private LineRenderer line;
    private LineRenderer[] lines=new LineRenderer[max_paths]; //caching the renderers so it doesn't have to getComponent on update
    public List<GameObject> paths = new List<GameObject>();

    private static Vector3[,] path_positions= new Vector3[max_paths,max_path_points];

    private static int[] path_lengths= new int[max_paths];

    private static float[] path_widths= new float[max_paths];
    private static Color[] path_colors= new Color[max_paths];
    // Start is called before the first frame update
    void Start()
    {
      StartCoroutine("Listener");
      StartSpheres();
      StartPaths();
      StartArrows();
    }

    // Update is called once per frame
    void Update()
    {
      UpdateSpheres();
      UpdatePaths();
      UpdateArrows();

    }

    private void StartSpheres(){ //Initialize sphere game objects
      for(int sphere_idx = 0; sphere_idx<max_spheres; ++sphere_idx){
        sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        sphere.transform.position = default_sphere_position;
        sphere.transform.localScale = default_sphere_scale;
        sphere_renderers[sphere_idx]=sphere.GetComponent<Renderer>();
        sphere_renderers[sphere_idx].material=new Material(Shader.Find("Sprites/Default"));
        sphere_renderers[sphere_idx].material.color=default_sphere_color;
        Destroy(sphere.GetComponent<SphereCollider>());
        //sphere.transform.SetParent(gameObject.transform);
        sphere.transform.SetParent(gameObject.transform.Find("Spheres"));
        spheres.Add(sphere);
      }
    }

    private void StartPaths(){ //Initialize path game objects
      for(int path_idx = 0; path_idx<max_paths; ++path_idx){
        path = new GameObject("path"); //gameObject.AddComponent<LineRenderer>();
        lines[path_idx] = path.AddComponent<LineRenderer>();// lineRenderer.material = new Material(Shader.Find("Sprites/Default"));
        lines[path_idx].widthMultiplier = 0.01f;
        lines[path_idx].positionCount = 0;
        //Set the paths default material/color
        lines[path_idx].material = new Material(Shader.Find("Sprites/Default"));
        lines[path_idx].material.color = default_path_color;
        path.transform.SetParent(gameObject.transform.Find("Paths"));
        paths.Add(path);
      }
    }

    private void UpdateSpheres(){
      //Clear spheres -> move spheres from previous frame to default location (beneath the map)
      for(int sphere_idx = 0; sphere_idx<last_sphere_count; ++sphere_idx){
        spheres[sphere_idx].transform.position = default_sphere_position;
        spheres[sphere_idx].transform.localScale = default_sphere_scale;
      }
      // Update spheres with LCM information
      if(sphere_count>0){
        for(int sphere_idx = 0; sphere_idx<sphere_count; ++sphere_idx){
          spheres[sphere_idx].transform.position = sphere_positions[sphere_idx]; //new Vector3(sphere_positions[sphere_idx,0], sphere_positions[sphere_idx,2], sphere_positions[sphere_idx,1]);
          spheres[sphere_idx].transform.localScale = sphere_scales[sphere_idx]; //new Vector3(sphere_radii[sphere_idx],sphere_radii[sphere_idx],sphere_radii[sphere_idx]);
          sphere_renderers[sphere_idx].material.color=sphere_colors[sphere_idx];
        }
      }
      last_sphere_count=sphere_count;

    }

    private void UpdatePaths(){
      // Clear Paths -> Reset the path count from paths in previous frames
      for(int path_idx = 0; path_idx<last_path_count; ++path_idx){
        lines[path_idx].positionCount = 0;
      }
      // Update Paths with LCM information
      if(path_count>0){
        for(int path_idx = 0; path_idx<path_count; ++path_idx){
          lines[path_idx].positionCount = path_lengths[path_idx];
          Vector3[] current_path_positions= new Vector3[path_lengths[path_idx]]; //create array of current path_positions(inefficient but good for now)
          for(int point_idx=0;point_idx<path_lengths[path_idx]; ++point_idx){
            current_path_positions[point_idx]=path_positions[path_idx,point_idx];
          }
          lines[path_idx].SetPositions(current_path_positions); //assign positions
          //change color
          lines[path_idx].material.color=path_colors[path_idx];
          lines[path_idx].widthMultiplier=path_widths[path_idx];
        }
      }
      last_path_count=path_count;
    }


    private void StartArrows(){
      //array for game objects
      force1 = this.gameObject.transform.GetChild(0).gameObject;
      force2 = this.gameObject.transform.GetChild(1).gameObject;
      force3 = this.gameObject.transform.GetChild(2).gameObject;
      force4 = this.gameObject.transform.GetChild(3).gameObject;
    }

    private void UpdateArrows(){
      // reshape arrays from left handed Unity convention for position
      // could make this more efficient if framerate becomes an issue, will fix if needed - sehwan
      for (int leg = 0; leg < 4; ++leg){
          unityPositionReshape[0] = (float)p_contact[leg, 0];
          unityPositionReshape[1] = (float)(p_contact[leg, 2]);
          unityPositionReshape[2] = (float)p_contact[leg, 1];
          unityForceReshape[0] = (float)f_contact[leg, 0];
          unityForceReshape[1] = (float)f_contact[leg, 1];
          unityForceReshape[2] = (float)f_contact[leg, 2];
          positions[leg] = unityPositionReshape;
          forces[leg] = unityForceReshape;
      }

      force1.transform.position = positions[0]; // moves force to point of contact
      scale1 = force1.transform.localScale; // scaling vector
      scale1.y = forces[0].magnitude/scaleFactor; // scales force arrow lengthwise
      force1.transform.localScale = scale1;
      if (forces[0]!=Vector3.zero){ // if in contact
        force1.transform.rotation = Quaternion.LookRotation(forces[0]);} // orients force vector in same direction as contact force

      force2.transform.position = positions[1];
      scale2 = force2.transform.localScale;
      scale2.y = forces[1].magnitude/scaleFactor;
      force2.transform.localScale = scale2;
      if (forces[1]!=Vector3.zero){
        force2.transform.rotation = Quaternion.LookRotation(forces[1]);}

      force3.transform.position = positions[2];
      scale3 = force3.transform.localScale;
      scale3.y = forces[2].magnitude/scaleFactor;
      force3.transform.localScale = scale3;
      if (forces[2]!=Vector3.zero){
        force3.transform.rotation = Quaternion.LookRotation(forces[2]);}

      force4.transform.position = positions[3];
      scale4 = force4.transform.localScale;
      scale4.y = forces[3].magnitude/scaleFactor;
      force4.transform.localScale = scale4;
      if (forces[3]!=Vector3.zero){
        force4.transform.rotation = Quaternion.LookRotation(forces[3]);}
    }





    internal class SimpleSubscriber : LCM.LCM.LCMSubscriber
    {

      public void MessageReceived(LCM.LCM.LCM lcm, string channel, LCM.LCM.LCMDataInputStream dins)
      {
        // Debug.Log ("RECV: " + channel);

        if (channel == "debug_visualization"){

          LCMTypes.debug_visualization_lcmt msg = new LCMTypes.debug_visualization_lcmt(dins);
          // Force Arrows
          for(int link_id = 0; link_id<2; ++link_id){
            for(int j=0; j<3; ++j){
              p_contact[link_id, j] = msg.arrow_base_positions[3*link_id+j];
              f_contact[link_id, j] = msg.arrow_directions[3*link_id+j];
            }
          }

          // Spheres
          sphere_count=msg.sphere_count;
          if(msg.sphere_count>0){
            for(int sphere_idx = 0; sphere_idx<sphere_count; ++sphere_idx){
              sphere_positions[sphere_idx]=new Vector3(msg.sphere_positions[sphere_idx*3],
                                                        msg.sphere_positions[sphere_idx*3+2],
                                                        msg.sphere_positions[sphere_idx*3+1]);
              sphere_scales[sphere_idx]=new Vector3(msg.sphere_radii[sphere_idx],
                                                    msg.sphere_radii[sphere_idx],
                                                    msg.sphere_radii[sphere_idx]);
              sphere_colors[sphere_idx]=new Color(msg.sphere_colors[sphere_idx*4],
                                                    msg.sphere_colors[sphere_idx*4+1],
                                                    msg.sphere_colors[sphere_idx*4+2],
                                                    msg.sphere_colors[sphere_idx*4+3]);
            }
          }
          // Paths
          path_count=msg.path_count;
          if(msg.path_count>0){
            path_lengths=msg.path_lengths;
            path_widths=msg.path_widths;
            for(int path_idx = 0; path_idx<path_count; ++path_idx){
              //colors and sizes
              path_colors[path_idx]=new Color(msg.path_colors[path_idx*4],
                                              msg.path_colors[path_idx*4+1],
                                              msg.path_colors[path_idx*4+2],
                                              msg.path_colors[path_idx*4+3]);
              for(int point_idx=0;point_idx<msg.path_lengths[path_idx]; ++point_idx){
                path_positions[path_idx,point_idx]=new Vector3(msg.path_positions[msg.path_start_idxs[path_idx]+point_idx*3],
                                                                msg.path_positions[msg.path_start_idxs[path_idx]+point_idx*3+2],
                                                                msg.path_positions[msg.path_start_idxs[path_idx]+point_idx*3+1]);
              }
            }
          }
        }
      }
    }




    Boolean running = false;
    LCM.LCM.LCM myLCM = null;

    public IEnumerator Listener()
    {
      Debug.Log ("Debug listener started.");
      myLCM = new LCM.LCM.LCM();

      myLCM.Subscribe("debug_visualization", new SimpleSubscriber());
      // may want to subscribe only to one channel: myLCM.Subscribe("quadruped_visualization_info", new SimpleSubscriber());
      running = true;
      while (running){
        yield return null;
      }
      Debug.Log ("Debug listener coroutine returning!");
    }
  }
