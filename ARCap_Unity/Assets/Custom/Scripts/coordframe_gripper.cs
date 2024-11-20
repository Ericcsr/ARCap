using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.XR;
using UnityEngine.Rendering;
using UnityEngine.SceneManagement;
using Unity.XR.Oculus;
using Meta.XR.Depth;
using TMPro;
using UnityEditor;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Text;

public class CoordinateFrameGripper : MonoBehaviour
{
    // Start is called before the first frame update
    private bool isClicked = false;
    private bool isFreezeUpdate = false;
    GameObject frame;
    GameObject robot;
    GameObject robot_ee;
    GameObject hand;
    OVRCameraRig cameraRig;
    private Vector3 hand_offset = new Vector3(0.0f, 0.0f, 0.0f);
    private Quaternion hand_rot_offset = Quaternion.Euler(0f, 0f, 0f);
    Vector3 cum_dist = new Vector3(0, 0, 0);
    public Vector3 current_pos = new Vector3(0, 0, 0);
    Vector3 last_anchor_pos = new Vector3(0, 0, 0);
    public Quaternion cum_rot = new Quaternion(0, 0, 0, 1);
    public static Vector3 last_pos = new Vector3(0, 0, 0);
    public static Quaternion last_rot = new Quaternion(0, 0, 0, 1);
    public static string folder_path;
    public static string remote_ip;
    public static bool isInitialized = false;
    [SerializeField]
    public string pc_ip;
    [SerializeField]
    public string ws_ip = "";
    [SerializeField]
    public int sender_port = 12346;
    private Socket sender;
    private IPEndPoint targetEndPoint;
    private TextMeshProUGUI init_text;
    private TouchScreenKeyboard overlayKeyboard;
    private bool data_collection_mode = true;
    void Start()
    {
        if (!isInitialized)
        {
            isInitialized = true;
        }
        else
        {
            isFreezeUpdate = true;
            current_pos = MainDataRecorderGripper.last_pos;
            last_anchor_pos = current_pos;
            cum_rot = MainDataRecorderGripper.last_rot;
        }
        frame = GameObject.Find("coordinate");
        robot = GameObject.Find("panda_link0");
        robot_ee = GameObject.Find("panda_hand");
        hand = GameObject.Find("gripper_base");
        cameraRig = GameObject.Find("OVRCameraRig").GetComponent<OVRCameraRig>();
        folder_path = Application.persistentDataPath + "/" + DateTime.Now.ToString("yyyy-MM-dd-HH-mm-ss");
        Directory.CreateDirectory(folder_path);
        sender = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);
        init_text = GameObject.Find("InitialText").GetComponent<TextMeshProUGUI>();
        init_text.text = "Data collection, Y: Place robot";
        if(isInitialized)
        {
            init_text.text = "Adjust robot, X: save and continue";
        }
        pc_ip = StartScene.pc_ip;
        remote_ip = pc_ip;
        if (CoordinateFrame.isBimanual)
        {
            remote_ip = CoordinateFrame.remote_ip;
            if (remote_ip == pc_ip)
            {
                init_text.text = "Data collection, Y: Place robot";
            }
            else
            {
                init_text.text = "Deploy, Y: Place robot";
            }
        }
        //targetEndPoint = new IPEndPoint(IPAddress.Parse(pc_ip), sender_port);
    }

    void saveWorldFrame(Vector3 pos, Quaternion rot)
    {
        //string path = Application.persistentDataPath+"/WorldFrame.txt";
        string worldframe;
        if(!CoordinateFrame.isBimanual)
        {
            worldframe = "WorldFrame:" + pos.x + "," + pos.y + "," + pos.z + "," + rot.x + "," + rot.y + "," + rot.z + "," + rot.w;
        }
        else
        {
            worldframe = "RobotFrame:" + pos.x + "," + pos.y + "," + pos.z + "," + rot.x + "," + rot.y + "," + rot.z + "," + rot.w;
        }
        byte[] message = System.Text.Encoding.UTF8.GetBytes(worldframe);
        sender.SendTo(message, message.Length, SocketFlags.None, targetEndPoint);
    }
    // Update is called once per frame
    void Update()
    {
        cum_dist += new Vector3(0, 0.005f, 0) * OVRInput.Get(OVRInput.Axis2D.PrimaryThumbstick).y;
        cum_dist += new Vector3(0.005f, 0, 0) * OVRInput.Get(OVRInput.Axis2D.SecondaryThumbstick).x;
        cum_dist += new Vector3(0, 0, 0.005f) * OVRInput.Get(OVRInput.Axis2D.SecondaryThumbstick).y;
        cum_rot *= Quaternion.Euler(0, 1.0f * OVRInput.Get(OVRInput.Axis2D.PrimaryThumbstick).x , 0);
        if(!isFreezeUpdate)
        {
            
            last_anchor_pos = cameraRig.leftHandAnchor.position;
            
        }
        current_pos = last_anchor_pos + cum_dist;
        frame.transform.position = current_pos;
        frame.transform.rotation = cum_rot;
        robot.transform.position = current_pos;
        robot.transform.rotation = cum_rot;
        // Set Palm position to be the same as end effector of the robot
        // Should apply palm offset
        hand.transform.position = robot_ee.transform.position + robot_ee.transform.rotation * hand_rot_offset * hand_offset;
        hand.transform.rotation = robot_ee.transform.rotation * hand_rot_offset;
        if (OVRInput.GetUp(OVRInput.RawButton.Y))
        {
            isFreezeUpdate = !isFreezeUpdate;
            if(isFreezeUpdate)
            {
                init_text.text = "Adjust robot, X: save and continue";
            }
            else
            {
                if(remote_ip == pc_ip)
                {
                    init_text.text = "Data collection Y: Place robot";
                }
                else
                {
                    init_text.text = "Deploy Y: Place robot";
                }
            }
            
        }
        if (OVRInput.GetUp(OVRInput.RawButton.X))
        {
            isClicked = true;
            // Save world frame
            last_pos = current_pos;
            last_rot = cum_rot;
            if(data_collection_mode && !CoordinateFrame.isBimanual)
            {
                remote_ip = pc_ip;
            }
            else if (!CoordinateFrame.isBimanual)
            {
                remote_ip = ws_ip;
            }
            targetEndPoint = new IPEndPoint(IPAddress.Parse(remote_ip), sender_port);
            saveWorldFrame(last_pos, last_rot);
            sender.Close(); // Free the socket
        }
        if (OVRInput.GetUp(OVRInput.RawButton.B) && !CoordinateFrame.isBimanual)
        {
            data_collection_mode = true;
            if(!isFreezeUpdate)
            {
                init_text.text = "Data collection Y: Place robot";
            }
            else
            {
                init_text.text = "Adjust robot, X: save and continue";
            }
            

        }
        if (OVRInput.GetUp(OVRInput.RawButton.A) && !CoordinateFrame.isBimanual)
        {
            overlayKeyboard = TouchScreenKeyboard.Open("Enter Workstation IP", TouchScreenKeyboardType.Default);
            data_collection_mode = false;
            if(!isFreezeUpdate)
            {
                init_text.text = "Deploy Y: Place robot";
            }
            else
            {
                init_text.text = "Adjust robot, X: save and continue";
            }
            
        }
        if (isClicked)
        {
            isClicked = false;
            if(CoordinateFrame.isBimanual)
            {
                SceneManager.LoadScene("BimanualCollection");
            }
            else
            {
                SceneManager.LoadScene("GripperCollection");
            }
        }
        if(overlayKeyboard != null && overlayKeyboard.status == TouchScreenKeyboard.Status.Done)
        {
            ws_ip = overlayKeyboard.text;
        }
    }
}
