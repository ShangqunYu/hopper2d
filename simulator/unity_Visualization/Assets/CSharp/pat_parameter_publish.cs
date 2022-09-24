using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using LCM;

public class pat_parameter_publish : MonoBehaviour {

  LCM.LCM.LCM myLCM;
  LCMTypes.quadruped_parameters_lcmt msg = new LCMTypes.quadruped_parameters_lcmt();
  LCMTypes.quadruped_menu_data_lcmt menuMsg = new LCMTypes.quadruped_menu_data_lcmt();

  //float ltaxis = Input.GetAxis("XboxLeftTrigger");
  private Boolean push_button = false;
  public GameObject input_legend;

  // Use this for initialization
  void Start () {
    //Debug.Log("Publish start");
    myLCM = LCM.LCM.LCM.Singleton;
    //TODO: figure out if I want the menu to override the state of the system on visualization launch or open
    //uncomment this to make it reset upon opening the visualization, not just pressing esc the first time
    //MenuData.instance.LoadFromJson();
    MenuData.instance.gameObject.SetActive(false);
    input_legend = GameObject.Find("InputLegend");
    input_legend.SetActive(false);
  }

  void OnGUI(){
    Event e = Event.current;
    if (e.isKey)
    {
      Debug.Log("Detected key code: " + e.keyCode);
    }
  }

  // Update is called once per frame
  void Update () {
    //TimeSpan span = DateTime.Now - new DateTime(1970, 1, 1);
    push_button = false;

    if(Input.GetKeyDown(KeyCode.Alpha1) || Input.GetKeyDown(KeyCode.F1)){
      msg.control_mode = 1;
      push_button = true;
    }else if(Input.GetKeyDown(KeyCode.Alpha2) || Input.GetKeyDown(KeyCode.F2)){
    msg.control_mode = 2;
      push_button = true;
    }else if(Input.GetKeyDown(KeyCode.Alpha3) || Input.GetKeyDown(KeyCode.F3)){
      msg.control_mode = 3;
      push_button = true;
    }else if(Input.GetKeyDown(KeyCode.Alpha4) || Input.GetKeyDown(KeyCode.F4)){
      msg.control_mode = 4;
      push_button = true;
    }else if(Input.GetKeyDown(KeyCode.Alpha5) || Input.GetKeyDown(KeyCode.F5)){
      msg.control_mode = 5;
      push_button = true;
    }else if(Input.GetKeyDown(KeyCode.Alpha6) || Input.GetKeyDown(KeyCode.F6)){
      msg.control_mode = 6;
      push_button = true;
    }else if(Input.GetKeyDown(KeyCode.Alpha7) || Input.GetKeyDown(KeyCode.F7)){
      msg.control_mode = 7;
      push_button = true;
    }else if(Input.GetKeyDown(KeyCode.Alpha8) || Input.GetKeyDown(KeyCode.F8)){
      msg.control_mode = 8;
      push_button = true;
    }else if(Input.GetKeyDown(KeyCode.Alpha9) || Input.GetKeyDown(KeyCode.F9)){
      msg.control_mode = 9;
      push_button = true;
    }else if(Input.GetKeyDown(KeyCode.Alpha0) || Input.GetKeyDown(KeyCode.F10)){
      msg.control_mode = 12; //Vision (12)
      push_button = true;
    }else{
      //msg.control_mode = 0;
    }

    if (Input.GetKeyDown(KeyCode.Escape)){
      MenuData.instance.gameObject.SetActive(!MenuData.instance.gameObject.activeInHierarchy);
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
      // Forward & Backward
      if(Input.GetKeyUp(KeyCode.UpArrow) || Input.GetKeyUp(KeyCode.DownArrow)) {
        msg.key_vertical = 0.0f;
        push_button = true;
      } else if(Input.GetKeyDown(KeyCode.UpArrow)){
        Debug.Log("forward");
        msg.key_vertical = 1.0f;
        push_button = true;
      } else if(Input.GetKeyDown(KeyCode.DownArrow)){
        Debug.Log("backward");
        msg.key_vertical = -1.0f;
        push_button = true;
      }else {
        //msg.key_vertical = 0.0f;
      }

      // Right & Left
      if(Input.GetKeyUp(KeyCode.RightArrow) || Input.GetKeyUp(KeyCode.LeftArrow)) {
        msg.key_horizontal = 0.0f;
        push_button = true;
      } else if(Input.GetKeyDown(KeyCode.RightArrow)){
        Debug.Log("right");
        msg.key_horizontal = 1.0f;
        push_button = true;
      } else if(Input.GetKeyDown(KeyCode.LeftArrow)){
        Debug.Log("left");
        msg.key_horizontal = -1.0f;
        push_button = true;
      }else {
        //msg.key_horizontal = 0.0f;
      }

      // Yaw Turning
      if(Input.GetKeyUp(KeyCode.Period) || Input.GetKeyUp(KeyCode.Slash)){
        Debug.Log("Turn Zero");
        msg.key_turn = 0.0f;
        push_button = true;
      }else if(Input.GetKeyDown(KeyCode.Period)){
        msg.key_turn = -1.0f;
        push_button = true;
      }else if(Input.GetKeyDown(KeyCode.Slash)){
        msg.key_turn = 1.0f;
        push_button = true;
      }else{
        //msg.key_turn = 0.0f;
      }

      // Pitch
      if(Input.GetKeyUp(KeyCode.Semicolon) || Input.GetKeyUp(KeyCode.Quote)){
        msg.key_pitch = 0.0f;
        push_button = true;
      }else if(Input.GetKeyDown(KeyCode.Semicolon)){
        msg.key_pitch = 1.0f;
        push_button = true;
      }else if(Input.GetKeyDown(KeyCode.Quote)){
        msg.key_pitch = -1.0f;
        push_button = true;
      }else{
        //msg.key_pitch = 0.0f;
      }

      // Jump
      if(Input.GetKeyDown(KeyCode.J)){
        Debug.Log("jump");
        msg.jump_trigger = true;
        push_button = true;
      }else if(Input.GetKeyUp(KeyCode.J)){
        push_button = true;
        msg.jump_trigger = false;
      }
    }

    if(push_button){
      myLCM.Publish("quadruped_parameters", msg);
      myLCM.Publish("quadruped_menu_data", menuMsg);
    }
    //TODO: break this into a separate function
    if(MenuData.instance.isChanged()){
      Debug.Log("menu is changed");

      menuMsg.control_mode = MenuData.instance.get_control_mode();
      menuMsg.cheater_mode = MenuData.instance.get_cheater_mode();
      menuMsg.use_wbc = MenuData.instance.get_use_wbc();
      menuMsg.use_rc = MenuData.instance.get_use_rc();

      convertVector3toArray(menuMsg.Kd_body,MenuData.instance.get_Kd_body());
      //Added Swing gains
      convertVector3toArray(menuMsg.Kp_joint,MenuData.instance.get_Kp_joint());
      convertVector3toArray(menuMsg.Kd_joint,MenuData.instance.get_Kd_joint());

      MenuData.instance.change();
      myLCM.Publish("quadruped_menu_data", menuMsg);
    }
  }

  private void convertVector3toArray(double[] array, Vector3 vector){
    for(int i = 0; i < 3; i++) array[i] = vector[i];
  }
}
