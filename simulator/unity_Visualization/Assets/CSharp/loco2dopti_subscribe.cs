using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using LCM;

public class loco2dopti_subscribe : MonoBehaviour
{
    public Transform torso;

    public Transform right_link_foot;

    public Transform left_link_foot;
    //
    //protected Transform link_first;

    public static float[] torso_pos = new float[3];

    public static float[] right_foot_pos = new float[3];

    public static float[] left_foot_pos = new float[3];

    public static double[] floor = new double[1000];
    public static GameObject[] boxes;
    public static GameObject backbox;
    public static float terrain_density = 0.1f;
    public static float get_new_terrain;

    // Start is called before the first frame update
    void Start()
    {
        get_new_terrain = 0f;
        StartCoroutine("Listener");
        this.torso = this.transform.GetChild(0);
        this.right_link_foot = this.transform.GetChild(1);
        this.left_link_foot = this.transform.GetChild(2);

        boxes = new GameObject[500]; // Change the size as per your requirement
        backbox = new GameObject();
        backbox.transform.position = new Vector3(-5f, -0.05f, 0); // Change the position as per your requirement
        backbox.transform.localScale = new Vector3(10f, 0.1f, 2f); // Change the scale as per your requirement

        for (int i = 0; i < boxes.Length; i++) {
            boxes[i] = GameObject.CreatePrimitive(PrimitiveType.Cube);
            boxes[i].transform.position = new Vector3(i * terrain_density + terrain_density/2, -1.05f, 0); // Change the position as per your requirement
            boxes[i].transform.localScale = new Vector3(terrain_density, 0.1f, 2f); // Change the scale as per your requirement
        }

    }

    // Update is called once per frame
    void Update()
    {
        this.torso.localPosition = new Vector3(torso_pos[0],torso_pos[1], 0);
        this.torso.localRotation = Quaternion.Euler(0, 0, (float)(torso_pos[2]*180/Math.PI));

        this.right_link_foot.localPosition = new Vector3(right_foot_pos[0],right_foot_pos[1],0);
        this.right_link_foot.localRotation = Quaternion.Euler(0, 0, (float) 90);

        this.left_link_foot.localPosition = new Vector3(left_foot_pos[0],left_foot_pos[1],0);
        this.left_link_foot.localRotation = Quaternion.Euler(0, 0, (float) 90);  

        if (get_new_terrain == 1f){
            
            
            for (int i = 0; i < boxes.Length; i++) {
                if (floor[i]==1){
                    boxes[i].transform.position = new Vector3(i * terrain_density + terrain_density/2, - 0.05f, 0); 
                } else {
                    Debug.Log (i);
                    boxes[i].transform.position = new Vector3(i * terrain_density + terrain_density/2, - 1.05f, 0); 
                }
                
            }
            get_new_terrain = 0f;
        }


    }


    internal class SimpleSubscriber : LCM.LCM.LCMSubscriber
    {

        public void MessageReceived(LCM.LCM.LCM lcm, string channel, LCM.LCM.LCMDataInputStream dins)
        {
            // Debug.Log ("RECV: " + channel);
            if (channel == "loco2dOpti")
            {
                LCMTypes.loco2dOpti_lcmt msg = new LCMTypes.loco2dOpti_lcmt(dins);
                
                torso_pos[0] = (float)msg.body_pos[0];
                torso_pos[1] = (float)msg.body_pos[1];
                torso_pos[2] = (float)msg.theta;


                right_foot_pos[0] = (float)msg.r_contact_pos;
                if (msg.r_under_contact ==1){
                    right_foot_pos[1] = 0;
                }else{
                    right_foot_pos[1] = -5f;
                }
                // Debug.Log("right_foot_pos on x" + right_foot_pos[0]);
                
                right_foot_pos[2] = 0;


                left_foot_pos[0] = (float)msg.l_contact_pos;
                if (msg.l_under_contact ==1){
                    left_foot_pos[1] = 0;
                }else{
                    left_foot_pos[1] = -5f;
                }
                // Debug.Log("left_foot_pos on x" + left_foot_pos[0]);
                
                left_foot_pos[2] = 0;

            }

            if (channel == "terrain") {
                
                Debug.Log("get_new_terrain: " + get_new_terrain);
                LCMTypes.terrain_lcmt msg = new LCMTypes.terrain_lcmt(dins);
                String message = "Received message of the type terrain:\n ";
                Debug.Log (message);
                for(int i = 0; i<500; ++i){
                    floor[i] = msg.floor[i];
                }
                get_new_terrain = 1f;

            }

        }
    }
    bool running = false;
    LCM.LCM.LCM myLCM = null;

    public IEnumerator Listener()
    {
        Debug.Log ("Listener started.");
        myLCM = new LCM.LCM.LCM();

        myLCM.SubscribeAll(new SimpleSubscriber());
        running = true;
        while (running){
            yield return null;
        }
        Debug.Log ("listener coroutine returning!");
    }

}
