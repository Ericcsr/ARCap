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


public class MainDataRecorder : MonoBehaviour
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
    private Quaternion rotateX = Quaternion.Euler(-10f, -25f,-25f);
    private Quaternion rotateXinv = Quaternion.Euler(-30f, 25f, 25f);
    private Vector3 right_pos_offset = new Vector3(0.08f, 0.01f, -0.05f);
    private Vector3 left_pos_offset = new Vector3(-0.1f, -0.04f, -0.07f);
    private Vector3 rleap_pos_offset = new Vector3(0.05f, 0.1f, 0.05f);
    private Quaternion rleap_rot_offset = Quaternion.Euler(0f, 0f, 180f);
    private GameObject robot;
    private GameObject rleap;
    private GameObject robot_vis;
    private GameObject leap_vis;
    private GameObject robot_ee;
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
    [System.Serializable]
    private struct RokokoHand
    {
        public float[] lt;
        public float[] li;
        public float[] lm;
        public float[] lr;
        public float[] rt;
        public float[] ri;
        public float[] rm;
        public float[] rr;
    }
    RokokoHand hand;
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
        // robot.transform.position = CoordinateFrame.current_pos;
        // robot.transform.rotation = CoordinateFrame.cum_rot;
        // robot_base.transform.position = CoordinateFrame.current_pos;
        // robot_base.transform.rotation = CoordinateFrame.cum_rot;
        // rleap.GetComponent<ArticulationBody>().TeleportRoot(robot_ee.transform.position + robot_ee.transform.rotation * rleap_rot_offset * rleap_pos_offset, 
        //                                                     robot_ee.transform.rotation * rleap_rot_offset);
        // rleap.transform.position = robot_ee.transform.position + robot_ee.transform.rotation * rleap_rot_offset * rleap_pos_offset;
        // rleap.transform.rotation = robot_ee.transform.rotation * rleap_rot_offset;
        rleap.GetComponent<ArticulationBody>().TeleportRoot(robot_ee.transform.position + robot_ee.transform.rotation * rleap_rot_offset * rleap_pos_offset, robot_ee.transform.rotation * rleap_rot_offset);
        
            // Should use left eye anchor pose from OVRCameraRig
        var headPose = cameraRig.centerEyeAnchor.position;
        var headRot = cameraRig.centerEyeAnchor.rotation; // Should store them separately. [w,x,y,z]
        m_TimeText.enabled = true;
        if (client.Available > 0)
        {
            hand = GetMessage();
        }
        // Left controller on right hand, inversed
        Vector3 rightWristPos = cameraRig.leftHandAnchor.position + cameraRig.leftHandAnchor.rotation * left_pos_offset;
        Vector3 leftWristPos = cameraRig.rightHandAnchor.position + cameraRig.rightHandAnchor.rotation * right_pos_offset;
        Quaternion rightWristRot = cameraRig.leftHandAnchor.rotation * rotateXinv * rotateZXinv;
        Quaternion leftWristRot = cameraRig.rightHandAnchor.rotation * rotateX * rotateZX;
        updateVisSpheres(hand, leftWristPos, leftWristRot, rightWristPos, rightWristRot);
        // if time gap > 0.05 send hand pose
        if (Time.time - current_time > 0.02)
        {
            if(startRecording)
            {
                SendHeadWristPose("R", "Y", rightWristPos, rightWristRot, headPose, headRot);
            }
            else
            {
                SendHeadWristPose("R", "N", rightWristPos, rightWristRot, headPose, headRot);
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
        string message = record + handedness+"Hand:" + wrist_pos.x + "," + wrist_pos.y + "," + wrist_pos.z + "," + wrist_rot.x + "," + wrist_rot.y + "," + wrist_rot.z + "," + wrist_rot.w;
        message = message + "," + head_pos.x + "," + head_pos.y + "," + head_pos.z + "," + head_orn.x + "," + head_orn.y + "," + head_orn.z + "," + head_orn.w;
        byte[] data = Encoding.UTF8.GetBytes(message);
        sender.SendTo(data, data.Length, SocketFlags.None, targetEndPoint);
    }   
    
    private RokokoHand GetMessage()
    {
        string message = "";
        RokokoHand rokokoHand;
        try
        {
            byte[] receivedBytes = client.Receive(ref remoteEndPoint);
            message = Encoding.UTF8.GetString(receivedBytes);
            rokokoHand = JsonUtility.FromJson<RokokoHand>(message);
            //m_TimeText.text = "Received: " + rokokoHand.lt[2].ToString();
        }
        catch (Exception e)
        {
            m_TimeText.text = "Socket error: " + e.Message;
            rokokoHand = new RokokoHand();
        }
        return rokokoHand;
    }

    private void updateVisSpheres(RokokoHand hand, 
                                    Vector3 leftHandPos, Quaternion leftHandRot, 
                                    Vector3 rightHandPos, Quaternion rightHandRot)
    {
        spheres[0].transform.position = new Vector3(hand.lt[0], hand.lt[2], hand.lt[1]);
        spheres[1].transform.position = new Vector3(hand.li[0], hand.li[2], hand.li[1]);
        spheres[2].transform.position = new Vector3(hand.lm[0], hand.lm[2], hand.lm[1]);
        spheres[3].transform.position = new Vector3(hand.lr[0], hand.lr[2], hand.lr[1]);
        spheres[4].transform.position = new Vector3(hand.rt[0], hand.rt[2], hand.rt[1]);
        spheres[5].transform.position = new Vector3(hand.ri[0], hand.ri[2], hand.ri[1]);
        spheres[6].transform.position = new Vector3(hand.rm[0], hand.rm[2], hand.rm[1]);
        spheres[7].transform.position = new Vector3(hand.rr[0], hand.rr[2], hand.rr[1]);
        // transform each spheres to global position with left and right hand anchor
        for (int i = 0; i < 8; i++)
        {
            if (i < 4) // Left controller on right hand
                spheres[i].transform.position = leftHandPos + leftHandRot * spheres[i].transform.position;
            else
                spheres[i].transform.position = rightHandPos + rightHandRot * spheres[i].transform.position;
        }
        
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
        for (int i = 0; i < 8; i++)
        {
            GameObject sphere = GameObject.Find("Sphere"+i);
            sphere.transform.localScale = new Vector3(0.02f, 0.02f, 0.02f);
            sphere.transform.position = new Vector3(i, 0, 0);
            spheres[i] = sphere;
        }
        // Create a folder with current time
        folder_path = CoordinateFrame.folder_path;
        // Visualize coordinate frame pos
        robot = GameObject.Find("panda_link0_vis");
        robot_ee = GameObject.Find("panda_grasptarget_vis");
        rleap = GameObject.Find("rpalm_vis");
        GameObject frame = GameObject.Find("coordinate_vis");
        frame.transform.position = CoordinateFrame.last_pos;
        frame.transform.rotation = CoordinateFrame.last_rot;
        robot.GetComponent<ArticulationBody>().TeleportRoot(CoordinateFrame.last_pos, CoordinateFrame.last_rot);
        
        // Create sender socket
        sender = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);
        targetEndPoint = new IPEndPoint(IPAddress.Parse(CoordinateFrame.remote_ip), sender_port);
        // Initialize hand 
        hand = new RokokoHand();
        hand.lt = new float[3]{0.0f, 0.0f, 0.0f};
        hand.li = new float[3]{0.0f, 0.0f, 0.0f};
        hand.lm = new float[3]{0.0f, 0.0f, 0.0f};
        hand.lr = new float[3]{0.0f, 0.0f, 0.0f};
        hand.rt = new float[3]{0.0f, 0.0f, 0.0f};
        hand.ri = new float[3]{0.0f, 0.0f, 0.0f};
        hand.rm = new float[3]{0.0f, 0.0f, 0.0f};
        hand.rr = new float[3]{0.0f, 0.0f, 0.0f};
        current_time = Time.time;
        image_u = GameObject.Find("panel_r").GetComponent<Image>();
        image_r = GameObject.Find("panel_u").GetComponent<Image>();
        image_l = GameObject.Find("panel_l").GetComponent<Image>();
        image_b = GameObject.Find("panel_b").GetComponent<Image>();
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
        if(OVRInput.GetUp(OVRInput.RawButton.X))
        {
            robot_vis.SetActive(!robot_vis.activeSelf);
            leap_vis.SetActive(!leap_vis.activeSelf);
            image_b.enabled = !image_b.enabled;
            image_l.enabled = !image_l.enabled;
            image_r.enabled = !image_r.enabled;
            image_u.enabled = !image_u.enabled;
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