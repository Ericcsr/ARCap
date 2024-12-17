using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.XR;
using UnityEngine.Rendering;
using Unity.XR.Oculus;
using Meta.XR.Depth;
using TMPro;
using UnityEditor;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Text;


public class MainDataRecorderGripper : MonoBehaviour
{
    // #region Constants
    // private static readonly int VirtualDepthTextureID = Shader.PropertyToID("_CameraDepthTexture");
    // #endregion // Constants
    // Define some variables for program
    #region Private Variables
    OVRCameraRig cameraRig;
    private UdpClient client;
    private IPEndPoint remoteEndPoint;
    private Socket sender;
    private IPEndPoint targetEndPoint;
    private GameObject[] spheres = new GameObject[8];
    private Quaternion rotateZX = Quaternion.Euler(45f, 0f,90f);
    private Quaternion rotateZXinv = Quaternion.Euler(45f, 0f, -90f);
    private Quaternion rotateX = Quaternion.Euler(-20f, -25f,-25f); // left: positive ccw, negative cw
    private Quaternion rotateXinv = Quaternion.Euler(-30f, 25f, 25f);
    private Vector3 right_pos_offset = new Vector3(0.1f, -0.02f, -0.07f); //+/-: [down/up, right/left, front/back]
    private Vector3 left_pos_offset = new Vector3(0f,0f,0f);
    private Vector3 lgripper_pos_offset = new Vector3(0f, 0f, 0f);
    private Quaternion lgripper_rot_offset = Quaternion.Euler(0f, 0f, 0f);
    private GameObject robot;
    private GameObject rgripper;
    private GameObject robot_vis;
    private GameObject gripper_vis;
    private GameObject robot_ee;
    private OVRHand l_hand;
    private OVRSkeleton l_hand_skeleton;
    private string folder_path;
    private float current_time = 0.0f;
    // Some control flags
    private bool startRecording = false;
    private bool startRemoving = false;
    private bool deleted = false;
    private Image image_r;
    private Image image_l;
    private Image image_u;
    private Image image_b;
    #endregion


    #region Unity Inspector Variables
    // [SerializeField]
    // [Tooltip("The RawImage where the virtual depth map will be displayed.")]
    // private RawImage m_virtualDepthImage;
    [SerializeField]
    [Tooltip("Time text")]
    private TextMeshProUGUI m_TimeText;
    [SerializeField]
    public string local_ip;
    [SerializeField]
    public int listen_port = 65432;
    [SerializeField]
    public int sender_port = 12346;
    [SerializeField]
    public string handedness = "R";
    public static int traj_cnt = 0;

    #endregion // Unity Inspector Variables

    #region Private Methods
    /// <summary>
    /// Attempts to get any unassigned components.
    /// </summary>
    /// <returns>
    /// <c>true</c> if all components were satisfied; otherwise <c>false</c>.
    /// </returns>
    private bool TryGetComponents()
    {
        if (m_TimeText == null) { m_TimeText = GetComponent<TextMeshProUGUI>(); }
        return m_TimeText != null;
    }
    /// <summary>
    /// Attempts to show the depth textures.
    /// </summary>
    /// <returns>
    /// <c>true</c> if the textures were shown; otherwise <c>false</c>.
    /// </returns>
    private bool MainLoop()
    {
        // Attempt to get the global depth texture
        // This should be a image, get a image and send via redis?
        rgripper.GetComponent<ArticulationBody>().TeleportRoot(robot_ee.transform.position + robot_ee.transform.rotation * lgripper_rot_offset * lgripper_pos_offset, robot_ee.transform.rotation * lgripper_rot_offset);
        
            // Should use left eye anchor pose from OVRCameraRig
        var headPose = cameraRig.centerEyeAnchor.position;
        var headRot = cameraRig.centerEyeAnchor.rotation; // Should store them separately. [w,x,y,z]
        m_TimeText.enabled = true;
        // Left controller on right hand, inversed
        Vector3 leftWristPos = cameraRig.leftHandAnchor.position + cameraRig.leftHandAnchor.rotation * left_pos_offset;
        Vector3 rightWristPos = cameraRig.rightHandAnchor.position + cameraRig.rightHandAnchor.rotation * right_pos_offset;
        Quaternion leftWristRot = cameraRig.leftHandAnchor.rotation; //* rotateXinv * rotateZXinv;
        Quaternion rightWristRot = cameraRig.rightHandAnchor.rotation * rotateX * rotateZX;
        // updateVisSpheres(hand, leftWristPos, leftWristRot, rightWristPos, rightWristRot);
        // if time gap > 0.05 send hand pose
        if (Time.time - current_time > 0.02)
        {
            if(startRecording)
            {
                if (handedness == "L")
                {
                    SendHeadWristPose("L", "Y", leftWristPos, leftWristRot, headPose, headRot);
                }
                else
                {
                    SendHeadWristPose("L", "Y", leftWristPos, leftWristRot, headPose, headRot);
                }
            }
            else
            {
                if(handedness == "L")
                {
                    SendHeadWristPose("L", "N", leftWristPos, leftWristRot, headPose, headRot);
                }
                else
                {
                    SendHeadWristPose("R", "N", rightWristPos, rightWristRot, headPose, headRot);
                }
            }
            current_time = Time.time;
        }
        if (startRecording)
        {
            m_TimeText.text = "Recording:" + traj_cnt;
            // RecordData(counter++, headPose, headRot, 
            //             leftWristPos, leftWristRot,
            //             rightWristPos, rightWristRot,
            //             hand, current_time);
        }
        else
        {
            m_TimeText.text = "Not recording";
        }
        return true;
    }

    // handedness: "L" or "R"
    // record: "Y" or "N"
    private void SendHeadWristPose( string handedness, string record, Vector3 wrist_pos, Quaternion wrist_rot, Vector3 head_pos, Quaternion head_orn)
    {
        float isPinch = checkPinch();
        string message = record + handedness+"Hand:" + wrist_pos.x + "," + wrist_pos.y + "," + wrist_pos.z + "," + wrist_rot.x + "," + wrist_rot.y + "," + wrist_rot.z + "," + wrist_rot.w;
        message = message + "," + head_pos.x + "," + head_pos.y + "," + head_pos.z + "," + head_orn.x + "," + head_orn.y + "," + head_orn.z + "," + head_orn.w+","+isPinch;
        byte[] data = Encoding.UTF8.GetBytes(message);
        sender.SendTo(data, data.Length, SocketFlags.None, targetEndPoint);
    }   

    private float checkPinch()
    {
        Vector3 l_thumb_tip = l_hand_skeleton.Bones[(int)OVRSkeleton.BoneId.Hand_ThumbTip].Transform.position;
        Vector3 l_index_tip = l_hand_skeleton.Bones[(int)OVRSkeleton.BoneId.Hand_IndexTip].Transform.position;
        Vector3 l_middle_tip = l_hand_skeleton.Bones[(int)OVRSkeleton.BoneId.Hand_MiddleTip].Transform.position;
        float dist1 = Vector3.Distance(l_thumb_tip, l_index_tip);
        float dist2 = Vector3.Distance(l_thumb_tip, l_middle_tip);
        return (dist1 + dist2)/2;
    }

    #endregion // Private Methods

    #region Unity Message Handlers
    /// <summary>
    /// Start is called before the first frame update
    /// </summary>
    protected void Start()
    {
        // Load SelectWorldScene
        local_ip = StartScene.local_ip;
        cameraRig = GameObject.Find("OVRCameraRig").GetComponent<OVRCameraRig>();
        l_hand = GameObject.Find("LeftControllerAnchor").GetComponent<OVRHand>();
        l_hand_skeleton = GameObject.Find("LeftControllerAnchor").GetComponent<OVRSkeleton>();
        // Set depth map index 200 x 200, from 0 to 1 in x and y
        // for (int y = 0; y < 100; y++){
        //     for (int x = 0; x < 100; x++)
        //     {
        //         depth_coords.Add(new Vector2(x / 100f, y / 100f));
        //     }
        // }

        if (!TryGetComponents())
        {
            m_TimeText.text = "mising components";
            enabled = false;
        }
        // set up socket
        
        try
        {
            m_TimeText.text = "Socket connecting...";
            client = new UdpClient();
            client.Client.Bind(new IPEndPoint(IPAddress.Parse(local_ip), listen_port));
            //client.Client.Bind(new IPEndPoint(IPAddress.Any, 65432));
            remoteEndPoint = new IPEndPoint(IPAddress.Any, 0);
            m_TimeText.text = "Socket connected";
        }
        catch (Exception e)
        {
            m_TimeText.text = "Socket error: " + e.Message;
        }
        // Create 8 spheres for visualization
        for (int i = 0; i < 6; i++)
        {
            GameObject sphere = GameObject.Find("Sphere"+i);
            sphere.transform.localScale = new Vector3(0.02f, 0.02f, 0.02f);
            sphere.transform.position = new Vector3(i, 0, 0);
            spheres[i] = sphere;
        }
        m_TimeText.text = "Spheres found";
        // Create a folder with current time
        folder_path = CoordinateFrameGripper.folder_path;
        // Visualize coordinate frame pos

        robot = GameObject.Find("panda_link0_vis");
        robot_ee = GameObject.Find("panda_hand_vis");
        rgripper = GameObject.Find("gripper_base_vis");
        robot_vis = GameObject.Find("panda_vis");
        gripper_vis = GameObject.Find("gripper_vis");
        GameObject frame = GameObject.Find("coordinate_vis");
        m_TimeText.text = "Object found";
        frame.transform.position = CoordinateFrameGripper.last_pos;
        frame.transform.rotation = CoordinateFrameGripper.last_rot;
        robot.GetComponent<ArticulationBody>().TeleportRoot(CoordinateFrameGripper.last_pos, CoordinateFrameGripper.last_rot);
        
        // Create sender socket
        sender = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);
        targetEndPoint = new IPEndPoint(IPAddress.Parse(CoordinateFrameGripper.remote_ip), sender_port);
        // Initialize hand 
        m_TimeText.text = "Endpoint init";
        // hand = new RokokoHand();
        // hand.lt = new float[3]{0.0f, 0.0f, 0.0f};
        // hand.li = new float[3]{0.0f, 0.0f, 0.0f};
        // hand.lm = new float[3]{0.0f, 0.0f, 0.0f};
        // hand.lr = new float[3]{0.0f, 0.0f, 0.0f};
        // hand.rt = new float[3]{0.0f, 0.0f, 0.0f};
        // hand.ri = new float[3]{0.0f, 0.0f, 0.0f};
        // hand.rm = new float[3]{0.0f, 0.0f, 0.0f};
        // hand.rr = new float[3]{0.0f, 0.0f, 0.0f};
        current_time = Time.time;
        image_u = GameObject.Find("panel_r").GetComponent<Image>();
        image_r = GameObject.Find("panel_u").GetComponent<Image>();
        image_l = GameObject.Find("panel_l").GetComponent<Image>();
        image_b = GameObject.Find("panel_b").GetComponent<Image>();
        m_TimeText.text = "Initialized";
    }

    /// <summary>
    /// Update is called once per frame
    /// </summary>
    protected void Update()
    {
        // Attempt to show the render textures
        MainLoop();
        if (OVRInput.GetUp(OVRInput.RawButton.A))
        {
            startRecording = !startRecording;
            if (startRecording)
            {
                deleted = false;
                startRemoving = false;
                var head_pose = cameraRig.centerEyeAnchor.position;
                var head_rot = cameraRig.centerEyeAnchor.rotation;
                string message = "Start:"+head_pose.x+","+head_pose.y+","+head_pose.z+","+head_rot.x+","+head_rot.y+","+head_rot.z+","+head_rot.w;
                byte[] data = Encoding.UTF8.GetBytes(message);
                sender.SendTo(data, data.Length, SocketFlags.None, targetEndPoint);
                traj_cnt ++;
                image_r.color = new Color32(188, 12, 13, 100);
                image_b.color = new Color32(188, 12, 13, 100);
                image_u.color = new Color32(188, 12, 13, 100);
                image_l.color = new Color32(188, 12, 13, 100);
            }
            else
            {
                image_r.color = new Color32(12, 188, 13, 100);
                image_b.color = new Color32(12, 188, 13, 100);
                image_u.color = new Color32(12, 188, 13, 100);
                image_l.color = new Color32(12, 188, 13, 100);
            }
        }
        if(OVRInput.GetUp(OVRInput.RawButton.B))
        {
            if(traj_cnt > 0 && !deleted)
            {
                startRemoving = true;
                traj_cnt --;
                deleted = true;
            }
            if(startRecording)
            {
                startRecording = false;
                image_r.color = new Color32(12, 188, 13, 100);
                image_b.color = new Color32(12, 188, 13, 100);
                image_u.color = new Color32(12, 188, 13, 100);
                image_l.color = new Color32(12, 188, 13, 100);
            }
        }
        if (!startRecording)
        {
            if (startRemoving)
            {
                string message = "Remove";
                byte[] data = Encoding.UTF8.GetBytes(message);
                sender.SendTo(data, data.Length, SocketFlags.None, targetEndPoint);
            }
            else
            {
                string message = "Stop";
                byte[] data = Encoding.UTF8.GetBytes(message);
                sender.SendTo(data, data.Length, SocketFlags.None, targetEndPoint);
            }
        }
        
    }

    protected void OnApplicationQuit()
    {
        if (client != null)
        {
            client.Close();
        }
    }
    #endregion // Unity Message Handlers
}