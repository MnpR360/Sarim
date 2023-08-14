using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using uPLibrary.Networking.M2Mqtt;

public class UGVMQTT : MonoBehaviour
{



    public ClientM2 clientClass;

    //these values are taken by M2MqttUnityClient class
    [Header("MQTT broker configuration")]
    public string brokerAddress = "broker.hivemq.com";
    public int brokerPort = 1883;
    public bool isEncrypted = true;
    //--------------------------------------------------

    public Robot robot1 = new Robot(1, "dawood", "up", 1.0f, 25.054485321044923f, 55.38414001464844f, 90);



    float timeToSend;
    float waitTime = 1;
    // Start is called before the first frame update
    void Start()
    {
        timeToSend = Time.time + waitTime;


        clientClass.ConnectToServer();

        StartCoroutine(WaitForConnectionAndSubscribe());





    }

    private IEnumerator WaitForConnectionAndSubscribe()
    {
        yield return new WaitForSeconds(1.0f);

        // Now that the connection is established, subscribe to robot 1's topic
        clientClass.SubscribeTopics(robot1.GetTopicNameSub());
    }


    public void Command( )
    {

        List<Environment_Struct.Message> command = clientClass.GetEventMsg();

        if (command != null )
        {
            foreach (Environment_Struct.Message msg in command)
            {
                Debug.Log(msg.topic + " " + msg.message);

                switch (msg.topic)
                {
                    case "mission":


                        break;

                    default:

                        break;
                }

            }


        }


    }




    // Update is called once per 
    void Update()
    {
        if (clientClass != null)
        {
            if (Time.time > timeToSend)
            {
                timeToSend = Time.time + waitTime;

                CoordsConverter converter = GetComponent<CoordsConverter>();
                Vector2 xyCoordinates = new Vector2(transform.position.x, transform.position.z); // Replace with your own x and y coordinates
                Vector2 lonLat = converter.ConvertXZToLonLat(xyCoordinates);

                robot1.UpdatePos(lonLat, transform.eulerAngles.y);
               //Debug.Log(robot1.ToJson());


                clientClass.PublishMSG(robot1.GetTopicNamePub(), robot1.ToJson());



            }

            Command();


        }


    }
}
