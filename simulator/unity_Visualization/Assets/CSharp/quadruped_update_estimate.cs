using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using LCM;
//using csharp;
//using c_test_lcm;

public class quadruped_update_estimate : MonoBehaviour
{
  public static int num_link = 13;
  public Transform[] link_trans = new Transform[num_link];
  public static float[,] link_ori_quat = new float[num_link,4];
  public static float[,] link_pos = new float[num_link,3];

  // Start is called before the first frame update
  void Start()
  {
    StartCoroutine("Listener");
    for(int i=0; i<num_link; ++i){
      this.link_trans[i] = this.transform.GetChild(i); //body
    }
  }

  // Update is called once per frame
  void Update()
  {
    for(int link_idx = 0; link_idx<num_link; ++link_idx){
      this.link_trans[link_idx].localPosition =
        new Vector3(link_pos[link_idx, 0], link_pos[link_idx, 2], link_pos[link_idx, 1]);

      this.link_trans[link_idx].localRotation =
        new Quaternion(
            link_ori_quat[link_idx, 1],
            link_ori_quat[link_idx, 2],
            link_ori_quat[link_idx, 3],
            link_ori_quat[link_idx, 0]);
    }
  }

  internal class SimpleSubscriber : LCM.LCM.LCMSubscriber
  {

    public void MessageReceived(LCM.LCM.LCM lcm, string channel, LCM.LCM.LCMDataInputStream dins)
    {
      //Debug.Log ("RECV: " + channel);
      if (channel == "quadruped_est_visualization_info")
      {
        LCMTypes.quadruped_visualization_info_lcmt msg =
          new LCMTypes.quadruped_visualization_info_lcmt(dins);

        for(int link_id = 0; link_id<num_link; ++link_id){
          for(int j=0; j<3; ++j){
            link_pos[link_id, j] = msg.link_pos[link_id, j];
            link_ori_quat[link_id, j] = msg.link_ori_quat[link_id, j];
          }
          link_ori_quat[link_id, 3] = msg.link_ori_quat[link_id, 3];
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
    running = true;
    while (running){
      yield return null;
    }
    Debug.Log ("Quadruped listener coroutine returning!");
  }

}
