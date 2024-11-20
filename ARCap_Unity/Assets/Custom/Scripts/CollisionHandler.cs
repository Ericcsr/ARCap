using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using Unity.XR.Oculus;
using TMPro;

public class CollisionHandler : MonoBehaviour
{
    // Start is called before the first frame update
    #region Unity Inspector Variables
    #endregion
    private TextMeshProUGUI m_Text;
    private string current_text;
    private Image image_r;
    private Image image_l;
    private Image image_u;
    private Image image_b;
    private GameObject robot_vis;
    void Start()
    {
        m_Text = GameObject.Find("DisplayText").GetComponent<TextMeshProUGUI>();
        image_u = GameObject.Find("panel_r").GetComponent<Image>();
        image_r = GameObject.Find("panel_u").GetComponent<Image>();
        image_l = GameObject.Find("panel_l").GetComponent<Image>();
        image_b = GameObject.Find("panel_b").GetComponent<Image>();
    }

    private void OnTriggerStay(Collider other)
    {
        if(image_r.enabled)
        {
            image_r.color = new Color32(12, 188, 188, 200);
            image_b.color = new Color32(12, 188, 188, 200);
            image_u.color = new Color32(12, 188, 188, 200);
            image_l.color = new Color32(12, 188, 188, 200);
            MainDataRecorderGripper.score -= 1;
            OVRInput.SetControllerVibration(1, 1, OVRInput.Controller.RTouch);
            OVRInput.SetControllerVibration(0, 0, OVRInput.Controller.RTouch);    
        }
    }
    
    // private void OnCollisionExit(Collision other)
    // {
    //     image_r.color = new Color32(12, 188, 13, 200);
    //     image_b.color = new Color32(12, 188, 13, 200);
    //     image_u.color = new Color32(12, 188, 13, 200);
    //     image_l.color = new Color32(12, 188, 13, 200);
    // }

}
