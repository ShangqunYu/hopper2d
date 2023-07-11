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

    // Start is called before the first frame update
    void Start()
    {
        StartCoroutine("Listener");
        this.torso = this.transform.GetChild(0);
        this.right_link_foot = this.transform.GetChild(1);
        this.left_link_foot = this.transform.GetChild(2);

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
    }


    internal class SimpleSubscriber : LCM.LCM.LCMSubscriber
    {

        public void MessageReceived(LCM.LCM.LCM lcm, string channel, LCM.LCM.LCMDataInputStream dins)
        {
            Debug.Log ("RECV: " + channel);
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
                Debug.Log("right_foot_pos on x" + right_foot_pos[0]);
                
                right_foot_pos[2] = 0;


                left_foot_pos[0] = (float)msg.l_contact_pos;
                if (msg.l_under_contact ==1){
                    left_foot_pos[1] = 0;
                }else{
                    left_foot_pos[1] = -5f;
                }
                Debug.Log("left_foot_pos on x" + left_foot_pos[0]);
                
                left_foot_pos[2] = 0;


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
