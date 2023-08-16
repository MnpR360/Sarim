using M2MqttUnity;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using uPLibrary.Networking.M2Mqtt.Messages;
public class ClientM2 : M2MqttUnityClient
{

    public List<Environment_Struct.Message> eventMessages = new List<Environment_Struct.Message>();



    public void PublishMSG(string topicNamePub, string msgSend)
    {   
        client.Publish(topicNamePub, System.Text.Encoding.UTF8.GetBytes(msgSend), MqttMsgBase.QOS_LEVEL_EXACTLY_ONCE, false);
    }

    public List<Environment_Struct.Message> GetEventMsg() {

        if (eventMessages.Count > 0)
        {
            List<Environment_Struct.Message> msgs = new List<Environment_Struct.Message>(eventMessages);
            eventMessages.Clear();

            return msgs;

        }

        return null;



    }

    protected override void DecodeMessage(string topic, byte[] message)
    {

        Environment_Struct.Message holder = new Environment_Struct.Message();
        holder.topic = topic;
        holder.message = System.Text.Encoding.UTF8.GetString(message);

        eventMessages.Add(holder);        
    }



    private void ConectionPP()
    {
        base.Update(); // call ProcessMqttEvents()

        
    }
    public void ConnectToServer()
    {
        base.Connect();
    }

    protected override void Update()
    {
        ConectionPP();
    }

    public void SubscribeTopics(string topicNameSub)
    {
        
        client.Subscribe(new string[] { topicNameSub }, new byte[] { MqttMsgBase.QOS_LEVEL_EXACTLY_ONCE });
        Debug.Log("Sub to " + topicNameSub);
    }

    void UnsubscribeTopics(string topicNameSub)
    {
        client.Unsubscribe(new string[] { topicNameSub });
    }



    protected override void Start()
    {
        base.Start();
    }

}


