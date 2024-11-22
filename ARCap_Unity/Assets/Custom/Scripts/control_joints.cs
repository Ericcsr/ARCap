
using System;
using UnityEditor;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using System.Net;
using System.Net.Sockets;
using System.Threading.Tasks;
using TMPro;
public class JointController : MonoBehaviour
{
    // List of joints
    #region Unity Inspector Variables
    [SerializeField]
    public List<GameObject> joints = new List<GameObject>();
    [SerializeField]
    public GameObject root;
    [SerializeField]
    public List<float> targetPositions = new List<float>(); // Target position in degrees or meters depending on the joint type
    [SerializeField]
    public string local_ip;
    [SerializeField]
    public int client_port = 12345;
    [SerializeField]
    public bool data_collector = false;
    [SerializeField]
    public bool controlRoot = false;
    #endregion
    private List<ArticulationBody> articulationBodies = new List<ArticulationBody>();
    private UdpClient client;
    private IPEndPoint remoteEndPoint;
    List<float> jointCommands = new List<float>();
    List<float> rootCommands = new List<float>();
    List<float> rootInput = new List<float>();
    List<float> jointInput = new List<float>();
    private Image image_r;
    private Image image_l;
    private Image image_u;
    private Image image_b;
    private TextMeshProUGUI m_Text;
    private string current_txt = "";
    private bool updated = false;

    private Vector3 original_pos;
    private Quaternion original_rot;

    void Start()
    {
        //local_ip = StartScene.local_ip;
        local_ip = CoordinateFrameG1.local_ip;
        int i = 0;
        foreach (GameObject joint in joints)
        {
            ArticulationBody articulationBody = joint.GetComponent<ArticulationBody>();
            articulationBodies.Add(articulationBody);
            ArticulationDrive drive = articulationBody.xDrive;
            jointCommands.Add(targetPositions[i]);
            jointInput.Add(targetPositions[i]);
            drive.target = Mathf.Rad2Deg * targetPositions[i++]; // Set the target position
            articulationBody.xDrive = drive;
        }

        if (controlRoot){
            for (int j = 0; j< 6;j++){
                rootCommands.Add(0);
                rootInput.Add(0);
            }
            rootCommands.Add(1);
            rootInput.Add(1);
            original_pos = CoordinateFrameG1.last_pos;
            original_rot = CoordinateFrameG1.last_rot;
        }
        // Initialize UDP related variables
        client = new UdpClient();
        client.Client.Bind(new IPEndPoint(IPAddress.Parse(local_ip), client_port));
        remoteEndPoint = new IPEndPoint(IPAddress.Any, client_port);
        if (data_collector)
        {
            image_u = GameObject.Find("panel_r").GetComponent<Image>();
            image_r = GameObject.Find("panel_u").GetComponent<Image>();
            image_l = GameObject.Find("panel_l").GetComponent<Image>();
            image_b = GameObject.Find("panel_b").GetComponent<Image>();
            m_Text = GameObject.Find("DisplayText").GetComponent<TextMeshProUGUI>();
        }
    }

    private List<float> GetJointCommands()
    {
        jointCommands = new List<float>();
        byte[] data = client.Receive(ref remoteEndPoint);
        string[] commands = System.Text.Encoding.UTF8.GetString(data).Split(','); // A string separated by commas
        if (commands[0].Equals("N") && data_collector && image_r.enabled)
        {
            image_r.color = new Color32(188, 188, 13, 200);
            image_b.color = new Color32(188, 188, 13, 200);
            image_u.color = new Color32(188, 188, 13, 200);
            image_l.color = new Color32(188, 188, 13, 200);
            current_txt = m_Text.text;
            m_Text.text = "Moving too fast!";
        }
        else if (commands[0].Equals("Y") && data_collector && image_r.enabled)
        {
            image_r.color = new Color32(188, 12, 13, 100);
            image_b.color = new Color32(188, 12, 13, 100);
            image_u.color = new Color32(188, 12, 13, 100);
            image_l.color = new Color32(188, 12, 13, 100);
            if (m_Text.text.Equals("Moving too fast!"))
            {
                m_Text.text = current_txt;
            }
        }
        else if (commands[0].Equals("G") && data_collector && image_r.enabled)
        {
            image_r.color = new Color32(12, 188, 13, 100);
            image_b.color = new Color32(12, 188, 13, 100);
            image_u.color = new Color32(12, 188, 13, 100);
            image_l.color = new Color32(12, 188, 13, 100);
        }

        for (int i=0; i<targetPositions.Count; i++)
        {
            jointCommands.Add(float.Parse(commands[i+1]));
        }
        return jointCommands;
    }

    private async void GetJointCommandsAsync()
    {
        UdpReceiveResult result = await client.ReceiveAsync();
        byte[] data = result.Buffer;
        string[] commands = System.Text.Encoding.UTF8.GetString(data).Split(','); // A string separated by commas
        if (commands[0].Equals("N") && data_collector && image_r.enabled)
        {
            image_r.color = new Color32(188, 188, 13, 200);
            image_b.color = new Color32(188, 188, 13, 200);
            image_u.color = new Color32(188, 188, 13, 200);
            image_l.color = new Color32(188, 188, 13, 200);
            current_txt = m_Text.text;
            m_Text.text = "Moving too fast!";
        }
        else if (commands[0].Equals("Y") && data_collector && image_r.enabled)
        {
            image_r.color = new Color32(188, 12, 13, 100);
            image_b.color = new Color32(188, 12, 13, 100);
            image_u.color = new Color32(188, 12, 13, 100);
            image_l.color = new Color32(188, 12, 13, 100);
            if (m_Text.text.Equals("Moving too fast!"))
            {
                m_Text.text = current_txt;
            }
        }
        else if (commands[0].Equals("G") && data_collector && image_r.enabled)
        {
            image_r.color = new Color32(12, 188, 13, 100);
            image_b.color = new Color32(12, 188, 13, 100);
            image_u.color = new Color32(12, 188, 13, 100);
            image_l.color = new Color32(12, 188, 13, 100);
        }

        for (int i=0; i<targetPositions.Count; i++)
        {
            jointCommands[i] = float.Parse(commands[i+1]);
        }
        if (controlRoot){
            for (int i=targetPositions.Count; i<targetPositions.Count+7; i++)
            {
                rootCommands[i-targetPositions.Count] = float.Parse(commands[i+1]);
            }
        }
        updated = true;
    }

    void Update()
    {
        // Assuming the joint is a revolute joint with a single degree of rotational freedom\
        GetJointCommandsAsync();
        if (updated)
        {
            updated = false;
            for (int j = 0; j < jointCommands.Count; j++)
            {
                jointInput[j] = jointCommands[j]; // Set the target position
            }
            for (int j = 0; j < rootCommands.Count; j++)
            {
                rootInput[j] = rootCommands[j];
            }
        }
        int i = 0;
        foreach (ArticulationBody articulationBody in articulationBodies)
        {
            ArticulationDrive drive = articulationBody.xDrive;
            if (articulationBody.jointType == ArticulationJointType.RevoluteJoint)
            {
                drive.target = Mathf.Rad2Deg * jointInput[i++]; // Set the target position
            }
            else if (articulationBody.jointType == ArticulationJointType.PrismaticJoint)
            {
                drive.target = jointInput[i++]; // Set the target position
            }
            articulationBody.xDrive = drive;
        }

        // New come root commands are always delta compared to origin in Isaac Gym.
        i = 0;
        if(controlRoot)
        {
            Vector3 rootPos = new Vector3(rootInput[i++], rootInput[i++], rootInput[i++]);
            Quaternion rootRot = new Quaternion(rootInput[i++], rootInput[i++], rootInput[i++], rootInput[i++]);
            //root.GetComponent<ArticulationBody>().TeleportRoot(rootPos, rootRot*original_rot);
            root.transform.position = rootPos;
            root.transform.rotation = rootRot;
            m_Text.text = "robot pose updated!"+rootPos[0]+","+rootPos[1]+","+rootPos[2];
        }
    }
}