using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using LCM;

public class summer_state_update : MonoBehaviour
{
  public static int num_joint = 12;
  public Transform[] j_trans = new Transform[num_joint];
  public static float[] jpos = new float[num_joint];
  public static float[] body_pos = new float[3];
  public static float[] body_ori_quat = new float[4];

  void Start()
  {
    StartCoroutine("Listener");
    for(int i = 0; i<4; ++i){
    this.j_trans[3*i] = this.transform.GetChild(1).GetChild(i+2);
    this.j_trans[3*i+1] = this.transform.GetChild(1).GetChild(i+2).GetChild(2);
    this.j_trans[3*i+2] = this.transform.GetChild(1).GetChild(i+2).GetChild(2).GetChild(1);
    }
  }

  // Update is called once per frame
  void Update()
  {
    this.transform.localPosition =
      new Vector3(body_pos[0], body_pos[2], body_pos[1]);

    this.transform.localRotation = 
      new Quaternion(
          body_ori_quat[1],
          body_ori_quat[2],
          body_ori_quat[3],
          body_ori_quat[0]);


    Vector3 angle  = new Vector3(0f, 0f, 0f);
    for(int j_idx = 0; j_idx< num_joint; ++j_idx){
      angle = Vector3.zero;

      if(j_idx%3 == 0) {
        angle[2] = -jpos[j_idx]*180f/(float)Math.PI;
      }
      else {
        angle[0] = jpos[j_idx]*180f/(float)Math.PI;
      }

      this.j_trans[j_idx].localRotation = Quaternion.Euler(angle);
    }
  }

  internal class SimpleSubscriber : LCM.LCM.LCMSubscriber
  {

    public void MessageReceived(LCM.LCM.LCM lcm, string channel, LCM.LCM.LCMDataInputStream dins)
    {
      if (channel == "summer_state_info")
      {
        LCMTypes.mcvision_state_info_lcmt msg =
          new LCMTypes.mcvision_state_info_lcmt(dins);

        for (int i = 0; i<3; ++i){
          body_pos[i] = msg.body_pos[i];
          body_ori_quat[i] = msg.body_ori_quat_visual[i];
        }
        body_ori_quat[3] = msg.body_ori_quat_visual[3];

        for(int jidx = 0; jidx<num_joint; ++jidx){
          jpos[jidx] = msg.jpos[jidx];
        }
      }
    }
  }

  Boolean running = false;
  LCM.LCM.LCM myLCM = null;

  public IEnumerator Listener()
  {
    Debug.Log ("Quadruped listener started.");
    myLCM = new LCM.LCM.LCM();

    myLCM.SubscribeAll(new SimpleSubscriber());
    // may want to subscribe only to one channel: myLCM.Subscribe("quadruped_visualization_info", new SimpleSubscriber());
    running = true;
    while (running){
      yield return null;
    }
    Debug.Log ("Quadruped State Update listener coroutine returning!");
  }

}
