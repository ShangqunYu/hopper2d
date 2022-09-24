using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using LCM;

public class heightmap_update : MonoBehaviour
{

    public Transform body; // mini cheetah body
    public Terrain map;
    public static int heightmapResolution = 33; // Options: 33, 65, 129, 2*(res-1)-1, ..., 4097
    public static float[,] heights = new float[heightmapResolution, heightmapResolution];
    public static float[] mapCenter = new float[2];
    public static float[] mapSize = new float[3];
    public static float maxHeight = 1.0f;

    // Start is called before the first frame update
    void Start()
    {
        // Init LCM
        StartCoroutine("Listener");

        // Always use unit scale for terrain object
        transform.localScale = new Vector3(1, 1, 1);

        // Size of the heightmap in meters (x, z, y)
        map.terrainData.size = new Vector3(2, 0.1f, 2);

        // Resolution of the heightmap
        map.terrainData.heightmapResolution = heightmapResolution; 
        Debug.Log("[HEIGTMAP UPDATE] Heightmap Resolution: " + map.terrainData.heightmapResolution);

        // Question: What does detail resolution and detail resolution per patch mean?

        // How to make heightmap material transparent

        // Need to make sure that terrain width and length agree with heightmap dimensions...

        // In the future, when we simulate vision and load in terrains like hallways, we can use the sampleHeight function
        // on the Terrain object

    }


    // Update is called once per frame
    void Update()
    {
        // Question: What about the point cloud?

        // Test LCM subscribe
        // Debug.Log(total_cells);

        // Move heightmap with the robot body
        transform.position = new Vector3(mapCenter[0], 0.0f, mapCenter[1]) - 
                                new Vector3(0.5f*map.terrainData.size[0], 0.0f, 0.5f*map.terrainData.size[2]) ;
        //Debug.Log("[HEIGHTMAP UPDATE] Transformed map");

        // Set map size (might only need to do this once, after LCM connection has been established)
        map.terrainData.size = new Vector3(mapSize[0], mapSize[1], mapSize[2]);

        // Write to Unity heightmap
        map.terrainData.SetHeights(0, 0, heights);
        //Debug.Log("[HEIGHTMAP UPDATE] Set Heights");
    }


    internal class SimpleSubscriber : LCM.LCM.LCMSubscriber {

        public void MessageReceived(LCM.LCM.LCM lcm, string channel, LCM.LCM.LCMDataInputStream dins) {
            //Debug.Log ("RECV: " + channel);
            if (channel == "HEIGHTMAP") {
                LCMTypes.grid_map_lcmt msg = new LCMTypes.grid_map_lcmt(dins);

                // Size of heightmap (in meters)
                mapSize[0] = ( (float) msg.rows - 1.0f ) * (float) msg.resolution;
                mapSize[1] = ( (float) msg.cols - 1.0f ) * (float) msg.resolution;
                mapSize[2] = maxHeight;
                
                // Center of map position
                mapCenter[0] = (float) msg.center[0];
                mapCenter[1] = (float) msg.center[1];

                // Populate heightmap array
                for(int ix = 0; ix < heightmapResolution; ix++){        // world x direction
                    for (int iy = 0; iy < heightmapResolution; iy++){   // world y direction
                        int heightmap_indx_x;
                        int heightmap_indx_y;

                        // Need to add interpolation (rather than just taking floor)
                        heightmap_indx_x = (int) Math.Floor( (float) ix / (float) heightmapResolution * (float) msg.rows );
                        heightmap_indx_y = (int) Math.Floor( (float) iy / (float) heightmapResolution * (float) msg.cols );

                        heights[heightmapResolution - 1 - ix, heightmapResolution - 1 - iy] = (float) msg.heightmap[msg.rows * heightmap_indx_x + heightmap_indx_y];
                        if (heights[heightmapResolution - 1 - ix, heightmapResolution - 1 - iy] > 1.0f){
                            heights[heightmapResolution - 1 - ix, heightmapResolution - 1 - iy] = 1.0f;
                        } else if (heights[iy, ix] < 0.0f) {
                            heights[heightmapResolution - 1 - ix, heightmapResolution - 1 - iy] = 0.0f;
                        }

                    }
                }

            }

        }

    }

    Boolean running = false;
    LCM.LCM.LCM myLCM = null;

    public IEnumerator Listener() {
        Debug.Log ("Heightmap listener started.");
        myLCM = new LCM.LCM.LCM();

        myLCM.Subscribe("HEIGHTMAP", new SimpleSubscriber());
        running = true;
        while (running){
            yield return null;
        }
        Debug.Log ("Heightmap listener coroutine returning!");
    }

}
