using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using LCM;
//using csharp;
//using c_test_lcm;

public class teleop_state_update : MonoBehaviour
{
  public static int num_joint = 13;
  public Transform[] joint_trans = new Transform[num_joint];
  public static float[] jpos = new float[num_joint];
  public static float[] body_pos = new float[3];
  public static float[] body_ori_quat = new float[4];

  void Start()
  {
    StartCoroutine("Listener");


    this.joint_trans[0] = this.transform.GetChild(1);
    for(int i=1; i<7; ++i){
      this.joint_trans[i] = this.joint_trans[i-1].GetChild(1);
    }
    this.joint_trans[7] = this.joint_trans[5].GetChild(2);

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




      Vector3 angle = new Vector3(0f, 0f, 0f);
      //Base
      angle[1] = jpos[0]*180f/(float)Math.PI;
      this.joint_trans[0].localRotation = Quaternion.Euler(angle);

      //shoulder x
      angle[0] = jpos[1]*180f/(float)Math.PI;
      angle[1] = 0;
      this.joint_trans[1].localRotation = Quaternion.Euler(angle);

      //shoulder y
      angle[0] = 0;
      angle[1] = jpos[2]*180f/(float)Math.PI;
      this.joint_trans[2].localRotation = Quaternion.Euler(angle);

      //elbow
      angle[0] = jpos[3]*180f/(float)Math.PI;
      angle[1] = 0;
      this.joint_trans[3].localRotation = Quaternion.Euler(angle);

      //wrist pitch
      angle[0] = jpos[4]*180f/(float)Math.PI;
      angle[1] = 0;
      this.joint_trans[4].localRotation = Quaternion.Euler(angle);

      // Wrist Roll
      angle[0] = 0;
      angle[1] = jpos[5]*180f/(float)Math.PI;
      this.joint_trans[5].localRotation = Quaternion.Euler(angle);

      //gripper
      angle[1] = 0;
      angle[2] = jpos[6]*180f/(float)Math.PI;
      this.joint_trans[7].localRotation = Quaternion.Euler(angle);


  }

  internal class SimpleSubscriber : LCM.LCM.LCMSubscriber
  {

    public void MessageReceived(LCM.LCM.LCM lcm, string channel, LCM.LCM.LCMDataInputStream dins)
    {
      if (channel == "teleop_visualization_info")
      {
        LCMTypes.teleop_visualization_info_lcmt msg =
          new LCMTypes.teleop_visualization_info_lcmt(dins);

        for (int i = 0; i<3; ++i){
          body_pos[i] = msg.body_pos[i];
          body_ori_quat[i] = msg.body_ori_quat[i];
        }
        body_ori_quat[3] = msg.body_ori_quat[3];

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
    Debug.Log ("Teleop listener started.");
    myLCM = new LCM.LCM.LCM();

    myLCM.SubscribeAll(new SimpleSubscriber());
    // may want to subscribe only to one channel: myLCM.Subscribe("teleop_visualization_info", new SimpleSubscriber());
    running = true;
    while (running){
      yield return null;
    }
    Debug.Log ("Teleop State Update listener coroutine returning!");
  }

}
