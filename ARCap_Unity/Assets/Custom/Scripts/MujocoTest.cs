using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.XR;
using UnityEngine.Rendering;
using UnityEngine.SceneManagement;
using Unity.XR.Oculus;
using Meta.XR.Depth;
using UnityEditor;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Net.NetworkInformation;
using System.Text;
using Mujoco;

public class MujocoTest : MonoBehaviour
{
    // Start is called before the first frame update
    private int cnt = 0;
    private float time = 0.0f;
    void Start()
    {
        // Get Current mj scene
    }


    // Update is called once per frame
    unsafe void Update()
    {
        var data = MjScene.Instance.Data;
        var model = MjScene.Instance.Model;
        time += cnt * 0.1f;
        cnt++;
        data->qpos[10] = 10*Mathf.Sin(time);
    }
}
