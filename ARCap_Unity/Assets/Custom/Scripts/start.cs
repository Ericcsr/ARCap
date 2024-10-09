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
using System.Net.NetworkInformation;
using System.Text;

public class StartScene : MonoBehaviour
{
    // Start is called before the first frame update
    private bool isLeapHand = false;
    private bool isGripper = false;
    private bool isBimanual = false;
    OVRCameraRig cameraRig;
    private TextMeshProUGUI init_text;
    public static string pc_ip;
    public static string local_ip;

    private TouchScreenKeyboard overlayKeyboard;
    
    void Start()
    {
        init_text = GameObject.Find("StartText").GetComponent<TextMeshProUGUI>();
        GetLocalIPAddress();
        CoordinateFrame.isBimanual = false;
        overlayKeyboard = TouchScreenKeyboard.Open("Enter IP of your PC", TouchScreenKeyboardType.Default);
    }

    public void GetLocalIPAddress()
    {
        var host = Dns.GetHostEntry(Dns.GetHostName());
        foreach (var ip in host.AddressList)
        {
            if (ip.AddressFamily == AddressFamily.InterNetwork)
            {
                local_ip = ip.ToString();
            }
        }
    }


    // Update is called once per frame
    void Update()
    {
        if (overlayKeyboard != null && overlayKeyboard.status == TouchScreenKeyboard.Status.Done)
        {
            pc_ip = overlayKeyboard.text;
        }
        if (OVRInput.GetUp(OVRInput.RawButton.Y))
        {
            isLeapHand = !isLeapHand;
            if(isLeapHand)
            {
                init_text.text = "Leap hand selected, A: save and continue";
                CoordinateFrame.isBimanual = false;
            }
            else
            {
                init_text.text = "X: Gripper, Y: Leap hand, B: Bimanual";
            }
        }
        if (OVRInput.GetUp(OVRInput.RawButton.X))
        {
            isGripper = !isGripper;
            if(isGripper)
            {
                init_text.text = "Gripper selected, A: save and continue";
                CoordinateFrame.isBimanual = false;
            }
            else
            {
                init_text.text = "X: Gripper, Y: Leap hand, B: Bimanual";
            }
        }
        if (OVRInput.GetUp(OVRInput.RawButton.B))
        {
            isBimanual = !isBimanual;
            if(isBimanual)
            {
                init_text.text = "Bimanual selected, A: save and continue";
                CoordinateFrame.isBimanual = true;
            }
            else
            {
                init_text.text = "X: Gripper, Y: Leap hand, B: Bimanual";
                CoordinateFrame.isBimanual = false;
            }
        }
        if (OVRInput.GetUp(OVRInput.RawButton.A))
        {
            if (isLeapHand || isBimanual)
            {
                SceneManager.LoadScene("HandSelect");
            }
            else if (isGripper)
            {
                SceneManager.LoadScene("GripperSelect");
            }
        }
    }
}
