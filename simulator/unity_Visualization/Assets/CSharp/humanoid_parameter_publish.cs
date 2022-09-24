using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using LCM;

public class humanoid_parameter_publish : MonoBehaviour {

  LCM.LCM.LCM myLCM;
  LCMTypes.humanoid_parameters_lcmt msg = new LCMTypes.humanoid_parameters_lcmt();

  //float ltaxis = Input.GetAxis("XboxLeftTrigger");
  public GameObject input_legend;


  // Use this for initialization
  void Start () {
    //Debug.Log("Publish start");
    myLCM = LCM.LCM.LCM.Singleton;
    
    input_legend = GameObject.Find("InputLegend");
    input_legend.SetActive(false);
  }

  // Update is called once per frame
  void Update () {
    TimeSpan span = DateTime.Now - new DateTime(1970, 1, 1);
    Boolean push_button = false;
    if(Input.GetKeyDown(KeyCode.Alpha1)){
      msg.control_mode = 1;
      push_button = true;
    }else if(Input.GetKeyDown(KeyCode.Alpha2)){
      msg.control_mode = 2;
      push_button = true;
    }else if(Input.GetKeyDown(KeyCode.Alpha3)){
      msg.control_mode = 3;
      push_button = true;
    }else if(Input.GetKeyDown(KeyCode.Alpha4)){
      msg.control_mode = 4;
      push_button = true;
    }else if(Input.GetKeyDown(KeyCode.Alpha5)){
      msg.control_mode = 5;
      push_button = true;
    }else if(Input.GetKeyDown(KeyCode.Alpha6)){
      msg.control_mode = 6;
      push_button = true;
    }else if(Input.GetKeyDown(KeyCode.Alpha7)){
      msg.control_mode = 7;
      push_button = true;
    }else if(Input.GetKeyDown(KeyCode.Alpha8)){
      msg.control_mode = 8;
      push_button = true;
    }else if(Input.GetKeyDown(KeyCode.Alpha9)){
      msg.control_mode = 9;
      push_button = true;
    }else if(Input.GetKeyDown(KeyCode.Alpha0)){
      msg.control_mode = 10;
      push_button = true;
    }else{
      //msg.control_mode = 0;
    }

    if (Input.GetKeyDown(KeyCode.Escape)){
      input_legend.SetActive(!input_legend.activeInHierarchy);
    }

    if(Input.GetKeyDown(KeyCode.K)){
      msg.xbox_ctrl = !msg.xbox_ctrl;
    }

    if(msg.xbox_ctrl){
      msg.stick_left_horizontal = Input.GetAxis("Horizontal_1"); //left horizontal
      msg.stick_left_vertical = Input.GetAxis("Vertical_1"); // left vertical

      msg.stick_right_horizontal = Input.GetAxis("Horizontal_2"); // right vertical 
      msg.stick_right_vertical = Input.GetAxis("Vertical_2"); // right horizontal
      msg.jump_trigger = Input.GetButton("Fire1");

      push_button = true;

    //Debug.Log("one cycle");
    //Debug.Log(ltaxis);
    //Debug.Log(lhaxis);
    //Debug.Log(rtaxis);
    //Debug.Log(rhaxis);
    //Debug.Log("end one cycle");

    }else{ // Keyboard input
      // Up
      if(Input.GetKeyDown(KeyCode.UpArrow)){
        Debug.Log("up");
        msg.key_up = 1.0f;
        push_button = true;
      }else if(Input.GetKeyUp(KeyCode.UpArrow)) {
        push_button = true;
        msg.key_up = 0.0f;
      } 

      // Down
      if(Input.GetKeyDown(KeyCode.DownArrow)){
        Debug.Log("down");
        msg.key_down = 1.0f;
        push_button = true;
      }else if (Input.GetKeyUp(KeyCode.DownArrow)){
        push_button = true;
        msg.key_down = 0.0f;
      }

      // Right 
      if(Input.GetKeyDown(KeyCode.RightArrow)){
        Debug.Log("right");
        msg.key_right = 1.0f;
        push_button = true;
      }else if (Input.GetKeyUp(KeyCode.RightArrow)){
        push_button = true;
        msg.key_right = 0.0f;
      }

      // Left 
      if(Input.GetKeyDown(KeyCode.LeftArrow)){
        Debug.Log("left");
        msg.key_left = 1.0f;
        push_button = true;
      }else if(Input.GetKeyUp(KeyCode.LeftArrow)){
        push_button = true;
        msg.key_left = 0.0f;
      }
      // Jump
      if(Input.GetKey(KeyCode.J)){
        if(!msg.jump_trigger){
          Debug.Log("jump");
          push_button = true;
          msg.jump_trigger = true;
      }}else if(msg.jump_trigger){
          Debug.Log("stop jump");
          push_button = true;
          msg.jump_trigger = false;
      }
      // Turn Right
      if(Input.GetKey(KeyCode.I)){
        if(msg.turn_right != 1.0f){
          Debug.Log("turn right");
          push_button = true;
          msg.turn_right = 1.0f;
      }}else if(Input.GetKey(KeyCode.O)) {
        if(msg.turn_right != -1.0f){
          Debug.Log("turn left");
          push_button = true;
          msg.turn_right = -1.0f;
      }}else if(msg.turn_right != 0.0f){
          Debug.Log("not turning");
          push_button = true;
          msg.turn_right = 0.0f;
      }}
    if(push_button){
      myLCM.Publish("humanoid_parameters", msg);
    }
  }
}
