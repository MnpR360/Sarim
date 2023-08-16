using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;

public class UIScript : MonoBehaviour
{
    public TextMeshProUGUI currentPosText;
    public TextMeshProUGUI currentDisText;
    public Transform ugv;
    


    public void ChangeCurrentPos(string CP)
    {
        currentPosText.text = CP;
    }

    public void ChangeCurrentDis(string CP)
    {
        currentDisText.text = CP;
    }

     // Start is called before the first frame update
    void Start()
    {
       
    }

    // Update is called once per frame
    void Update()
    {
       // ChangeCurrentPos($"{ugv.position.x}, {ugv.position.z}");
    }
}
