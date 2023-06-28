using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using LCM;

public class hopper2dopti_subscribe : MonoBehaviour
{
    public Transform torso;

    public Transform link_foot;
    //
    //protected Transform link_first;

    public static float[] torso_pos = new float[3];

    public static float[] foot_pos = new float[3];

    // Start is called before the first frame update
    void Start()
    {
        StartCoroutine("Listener");
        this.torso = this.transform.GetChild(0);
        this.link_foot = this.transform.GetChild(1);
        //Debug.Log(this.transform.childCount);
        //this.link2 = this.transform.GetChild(1);
    }

    // Update is called once per frame
    void Update()
    {
        this.torso.localPosition = new Vector3(torso_pos[0],torso_pos[1], 0);
        this.torso.localRotation = Quaternion.Euler(0, 0, (float)(torso_pos[2]*180/Math.PI));

        this.link_foot.localPosition = new Vector3(foot_pos[0],foot_pos[1],0);
        this.link_foot.localRotation = Quaternion.Euler(0, 0, (float) 90);
         //this.link_first.Rotate(3.0f, (float)0.3f, link1_pos[2], Space.Self);
        //this.transform.GetChild(0).Rotate(0., 0., link1_pos[2], Space.Self); // = Quaternion.Euler(0., link1_pos[2], 0);
        //this.transform.Rotate(0., 0., 0., Space.Self); // = Quaternion.Euler(0., link1_pos[2], 0);
        //this.transform.GetChild(0).rotation = Quaternion.Euler(0., 0., 0.); 
        //link_first.Rotate(0., 0., link1_pos[2], Space.Self); // = Quaternion.Euler(0., link1_pos[2], 0);
        //this.transform.GetChild(0).rotation = new Quaternion.Euler(0., link1_pos[2],0);
        //this.transform.GetChild(0).rotation = Quaternion.Euler(0., link1_pos[2], 0); // Identifier error
        //transform.GetChild(0).rotation = Quaternion.Euler(0., this.link1_pos[2], 0); // Identifier error
        //transform.GetChild(0).rotation = new Quaternion.Euler(0., 30, 0); // Identifier error
        //transform.GetChild(0).rotation = new Quaternion((float)0.3, 0, 0, (float)1.3); // Identifier error
        
        //this.link_first.rotation = new Quaternion(0, 0, 0, 1) // O.K.;
        //this.link_first.rotation = new Quaternion((float)Math.Sin(0.523), 0, 0, (float)Math.Cos(0.523)); // O.K.
        //this.link_first.rotation = Quaternion.Euler(0, 0, 30); // O.K.
        //transform.GetChild(0).rotation = new Quaternion(0, 0, 0, 1); // O.K.
    }


    internal class SimpleSubscriber : LCM.LCM.LCMSubscriber
    {

        public void MessageReceived(LCM.LCM.LCM lcm, string channel, LCM.LCM.LCMDataInputStream dins)
        {
            Debug.Log ("RECV: " + channel);
            if (channel == "hopper2dOpti")
            {
                LCMTypes.hopper2dOpti_lcmt msg = new LCMTypes.hopper2dOpti_lcmt(dins);
                
                torso_pos[0] = (float)msg.body_pos[0];
                torso_pos[1] = (float)msg.body_pos[1];
                torso_pos[2] = (float)msg.theta;


                foot_pos[0] = (float)msg.contact_pos;
                if (msg.under_contact ==1){
                    foot_pos[1] = 0;
                }else{
                    foot_pos[1] = -5f;
                }
                Debug.Log("foot_pos on x" + foot_pos[0]);
                
                foot_pos[2] = 0;

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
